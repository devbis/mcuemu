// CC2530 five-channel DMA controller (SWRU191F Chapter 8).
//
// Model summary
// - Descriptors (SWRU191F 8.3, 8.6, Table 8-2) are fetched from XDATA with
//   full side effects when a channel is armed (DMAARMn 0 -> 1) and again on
//   every rearm of a repeated mode (Figure 8-1 loops back to "Load DMA
//   Channel Configuration"); SRCADDR/DESTADDR are copied to the internal
//   pointers at that time (8.2.6). Nothing is re-read while a channel stays
//   armed.
// - Arming takes 9 system clocks per channel, channels armed by one DMAARM
//   write are fetched sequentially: channel 1 first, channel 0 last, all
//   within the last clocks of the 9 x n window (8.1).
// - Each byte/word transfer is a data slot of the single DMA engine; every
//   slot is arbitrated separately (Figure 8-1: "Transfer one byte or word
//   when channel is granted access"; 8.2.8 priority and round-robin).
// - Data really moves through Bus::xdata_read/xdata_write.
// - Every Table 8-1 trigger is raised by its source, including DBG_BW (31),
//   which the debug interface raises for each BURST_WRITE byte it puts in
//   DBGDATA; the debug configuration bit DMA_PAUSE (Table 3-2) holds
//   arbitration (set_paused()).
// - The flash controller and the AES coprocessor are co-simulated in
//   advance() so their DMA triggers and the DMA accesses to FWDATA, ENCDI
//   and ENCDO happen at exact cycles in every run mode.
//
// All internal times are HF-clock times (Bus::hf_now()): the DMA halts with
// the HF clocks in PM1-PM3 (SWRU191F 4.x power modes) and resumes on wake.

#include "cc2530_dma.hpp"

#include "cc2530_aes.hpp"
#include "cc2530_flash.hpp"

#include <algorithm>

namespace mcs51emu::cc2530 {

namespace {

constexpr std::uint8_t SfrDmairq = 0xd1;
constexpr std::uint8_t SfrDma1cfgl = 0xd2;
constexpr std::uint8_t SfrDma1cfgh = 0xd3;
constexpr std::uint8_t SfrDma0cfgl = 0xd4;
constexpr std::uint8_t SfrDma0cfgh = 0xd5;
constexpr std::uint8_t SfrDmaarm = 0xd6;
constexpr std::uint8_t SfrDmareq = 0xd7;

constexpr std::uint8_t ChannelMask = 0x1f;
constexpr std::uint8_t ArmAbort = 0x80;
constexpr std::uint8_t ArmReserved = 0x60;

// SWRU191F 8.1: "the time to arm one channel (that is, get configuration
// data) takes nine system clocks".
constexpr std::uint64_t FetchCyclesPerChannel = 9;

// Model-defined: SWRU191F gives no per-transfer duration. A byte transfer
// occupies the DMA engine for 2 system clocks (one XDATA read and one XDATA
// write), a word transfer for 4.
constexpr unsigned ByteSlotCycles = 2;
constexpr unsigned WordSlotCycles = 4;

// TMODE values (Table 8-2).
constexpr std::uint8_t TmodeBlock = 1;
constexpr std::uint8_t TmodeRepeatedSingle = 2;
constexpr std::uint8_t TmodeRepeatedBlock = 3;

bool block_mode(std::uint8_t tmode) { return tmode == TmodeBlock || tmode == TmodeRepeatedBlock; }
bool repeated_mode(std::uint8_t tmode) {
    return tmode == TmodeRepeatedSingle || tmode == TmodeRepeatedBlock;
}

// PRIORITY (Table 8-2): 00 low, 01 assured, 10 high. 11 is reserved and is
// treated as high (counted as unmodeled when loaded).
unsigned priority_rank(std::uint8_t priority) { return priority >= 2 ? 2 : priority; }

// SRCINC/DESTINC (Table 8-2, 8.2.6): 0, +1, +2, -1 counts; a count is one
// byte in byte mode and two bytes in word mode.
std::uint16_t step(std::uint8_t increment, bool word) {
    const unsigned unit = word ? 2 : 1;
    switch (increment) {
    case 1:
        return static_cast<std::uint16_t>(unit);
    case 2:
        return static_cast<std::uint16_t>(2 * unit);
    case 3:
        return static_cast<std::uint16_t>(0x10000u - unit);
    default:
        return 0;
    }
}

} // namespace

Dma::Dma(Bus &bus) : Peripheral(bus) {
    for (const std::uint8_t sfr :
         {SfrDmairq, SfrDma1cfgl, SfrDma1cfgh, SfrDma0cfgl, SfrDma0cfgh, SfrDmaarm, SfrDmareq})
        bus_.claim_sfr(sfr, this, false);
}

void Dma::reset(ResetCause) {
    // All DMA registers reset to 0x00 (SWRU191F 8.8); the core zeroed storage.
    for (Channel &channel : channels_) {
        const std::uint32_t generation = channel.generation + 1;
        channel = Channel{};
        channel.generation = generation;
    }
    fetch_free_ = 0;
    engine_free_ = 0;
    slot_active_ = false;
    slot_channel_ = 0;
    slot_end_ = 0;
    slot_generation_ = 0;
    last_granted_ = Channels - 1;
    watermark_ = bus_.hf_now();
    cursor_ = watermark_;
    in_service_ = false;
}

std::uint8_t Dma::read(Reg reg) { return peek(reg); }

std::uint8_t Dma::peek(Reg reg) const { return bus_.storage(reg); }

void Dma::write(Reg reg, std::uint8_t value) {
    const std::uint64_t at = access_time();
    switch (sfr_of(reg)) {
    case SfrDmairq:
        // DMAIRQ: bits 4:0 R/W0, bits 7:5 R0 (SWRU191F 8.8).
        bus_.storage(reg) = static_cast<std::uint8_t>(bus_.storage(reg) & value & ChannelMask);
        break;
    case SfrDmaarm:
        write_arm(value, at);
        break;
    case SfrDmareq: {
        // DMAREQ: bits 4:0 R/W1 H0, bits 7:5 R0. Setting a bit "has the same
        // effect as a single trigger event"; it is cleared when the transfer
        // starts and not when the channel is disarmed (8.1, 8.8).
        const std::uint8_t set = static_cast<std::uint8_t>(value & ChannelMask);
        for (unsigned index = 0; index < Channels; ++index) {
            if (!(set & (1u << index)))
                continue;
            Channel &channel = channels_[index];
            if (!(bus_.storage(reg) & (1u << index)))
                channel.dmareq_at = at;
            // SWRU191F 8.1: "Enabling a software trigger (DMAREQ) clears
            // missed-trigger counting, and no new triggers are generated
            // while a new configuration is fetched from memory".
            channel.requests.clear();
            channel.early.clear();
        }
        bus_.storage(reg) = static_cast<std::uint8_t>(bus_.storage(reg) | set);
        break;
    }
    default:
        // DMA0CFGH/L, DMA1CFGH/L: plain R/W address registers, read when a
        // channel is armed or rearmed.
        bus_.storage(reg) = value;
        break;
    }
    if (!in_service_)
        bus_.reschedule();
}

std::uint64_t Dma::deadline() const {
    return bus_.hf_deadline(own_next_event());
}

void Dma::service(std::uint64_t) { advance(bus_.hf_now()); }

void Dma::power_mode_changed(PowerMode, PowerMode) {
    // The DMA runs on the HF clock: in PM1-PM3 hf_now() and every internal
    // time freeze and deadline() reports Never; transfers resume on wake.
    // DMA registers are retained in PM2/PM3 (SWRU191F 4.6).
}

void Dma::trigger(unsigned number) {
    // Trigger 0 (no trigger) and 1 (PREV, generated internally by channel
    // completion, SWRU191F Table 8-1) are never accepted from outside.
    if (number <= trigger::Prev || number > trigger::DbgBw)
        return;
    const std::uint64_t at = access_time();
    for (unsigned index = 0; index < Channels; ++index)
        record(index, number, at);
    if (!in_service_)
        bus_.reschedule();
}

std::uint64_t Dma::access_time() const {
    return in_service_ ? cursor_ : std::max(bus_.hf_now(), watermark_);
}

void Dma::record(unsigned index, unsigned number, std::uint64_t at) {
    Channel &channel = channels_[index];
    if (channel.state == State::Configuring) {
        // Sorted out against the old and the new TRIG when the fetch ends.
        channel.early.emplace_back(at, number);
    } else if (channel.state == State::Armed && channel.trig == number) {
        // Triggers arriving while the channel is busy are counted and served
        // in order ("missed events", SWRU191F 8.1).
        channel.requests.insert(
            std::upper_bound(channel.requests.begin(), channel.requests.end(), at), at);
    }
}

void Dma::set_arm_bit(unsigned index, bool armed) {
    std::uint8_t &dmaarm = bus_.storage(sfr_reg(SfrDmaarm));
    const auto bit = static_cast<std::uint8_t>(1u << index);
    dmaarm = static_cast<std::uint8_t>(armed ? dmaarm | bit : dmaarm & ~bit);
}

void Dma::write_arm(std::uint8_t value, std::uint64_t at) {
    std::uint8_t &dmaarm = bus_.storage(sfr_reg(SfrDmaarm));
    // Bits 6:5 are reserved R/W; bit 7 (ABORT) is R0/W.
    dmaarm = static_cast<std::uint8_t>((dmaarm & ChannelMask) | (value & ArmReserved));
    const auto selected = static_cast<std::uint8_t>(value & ChannelMask);
    if (value & ArmAbort) {
        abort(selected, at);
        return;
    }
    // DMAARMn is R/W1: writing 0 has no effect, arming an armed channel has
    // no effect.
    std::uint8_t fresh = 0;
    for (unsigned index = 0; index < Channels; ++index)
        if ((selected & (1u << index)) && channels_[index].state == State::Idle)
            fresh = static_cast<std::uint8_t>(fresh | (1u << index));
    dmaarm = static_cast<std::uint8_t>(dmaarm | selected);
    if (fresh)
        start_fetch(fresh, at);
}

void Dma::start_fetch(std::uint8_t channels, std::uint64_t at) {
    // SWRU191F 8.1: nine system clocks per channel; with all five armed it
    // takes 45 clocks, "channel 1 is ready first, then channel 2, and lastly
    // channel 0 (all within the last eight system clocks)".
    // Model-defined: channels armed together are fetched in the order
    // 1, 2, 3, 4, 0; with k channels the i-th (0-based) becomes ready at
    // start + 9k - (k - 1 - i), i.e. one clock apart at the end of the 9k
    // window. A new arm waits until the fetch unit is free.
    static constexpr unsigned Order[Channels] = {1, 2, 3, 4, 0};
    unsigned count = 0;
    for (const unsigned index : Order)
        if (channels & (1u << index))
            ++count;
    const std::uint64_t start = std::max(at, fetch_free_);
    const std::uint64_t window = FetchCyclesPerChannel * count;
    unsigned position = 0;
    for (const unsigned index : Order) {
        if (!(channels & (1u << index)))
            continue;
        Channel &channel = channels_[index];
        channel.state = State::Configuring;
        channel.ready_at = start + window - (count - 1 - position);
        channel.in_block = false;
        ++channel.generation;
        ++position;
    }
    fetch_free_ = start + window;
}

void Dma::load_descriptor(unsigned index, std::uint64_t at) {
    Channel &channel = channels_[index];
    // SWRU191F 8.3: channel 0 at DMA0CFGH:DMA0CFGL, channels 1-4 in a
    // contiguous 32-byte area at DMA1CFGH:DMA1CFGL.
    std::uint16_t base;
    if (index == 0)
        base = static_cast<std::uint16_t>(bus_.storage(sfr_reg(SfrDma0cfgh)) << 8 |
                                          bus_.storage(sfr_reg(SfrDma0cfgl)));
    else
        base = static_cast<std::uint16_t>(
            (bus_.storage(sfr_reg(SfrDma1cfgh)) << 8 | bus_.storage(sfr_reg(SfrDma1cfgl))) +
            8 * (index - 1));
    std::uint8_t d[8];
    const std::uint32_t generation = channel.generation;
    for (unsigned i = 0; i < 8; ++i)
        d[i] = bus_.xdata_read(static_cast<std::uint16_t>(base + i));
    if (channel.generation != generation || channel.state != State::Configuring)
        return; // the descriptor read itself aborted the channel

    const bool had_descriptor = channel.loaded;
    const std::uint8_t old_trig = channel.trig;
    const bool old_irqmask = channel.irqmask;

    // Table 8-2.
    channel.src = static_cast<std::uint16_t>(d[0] << 8 | d[1]);
    channel.dst = static_cast<std::uint16_t>(d[2] << 8 | d[3]);
    channel.vlen = static_cast<std::uint8_t>(d[4] >> 5);
    channel.len = static_cast<std::uint16_t>((d[4] & 0x1f) << 8 | d[5]);
    channel.word = (d[6] & 0x80) != 0;
    channel.tmode = static_cast<std::uint8_t>(d[6] >> 5 & 3);
    channel.trig = static_cast<std::uint8_t>(d[6] & 0x1f);
    channel.srcinc = static_cast<std::uint8_t>(d[7] >> 6);
    channel.destinc = static_cast<std::uint8_t>(d[7] >> 4 & 3);
    channel.irqmask = (d[7] & 0x08) != 0;
    channel.m8 = (d[7] & 0x04) != 0;
    channel.priority = static_cast<std::uint8_t>(d[7] & 3);
    channel.loaded = true;
    if (channel.vlen == 5 || channel.vlen == 6) // reserved VLEN: LEN is used
        bus_.note_unmodeled(sfr_reg(SfrDmaarm), true);
    if (channel.priority == 3) // reserved PRIORITY: treated as high
        bus_.note_unmodeled(sfr_reg(SfrDmaarm), true);

    // SWRU191F 8.2.6: pointers are loaded when the channel is armed/rearmed.
    channel.src_ptr = channel.src;
    channel.dst_ptr = channel.dst;
    channel.done = 0;
    channel.in_block = false;
    channel.total = channel.len;
    channel.total_known = channel.len == 0 || channel.vlen == 0 || channel.vlen >= 5;
    channel.state = State::Armed;
    channel.armed_at = at;

    // SWRU191F 8.1: triggers of the previously configured source that arrive
    // during the fetch are missed events served as soon as the channel is
    // ready; a trigger of the new source within the fetch time is lost.
    for (const auto &[time, number] : channel.early) {
        const bool missed_old = time < at && had_descriptor && number == old_trig;
        const bool new_source = time >= at && number == channel.trig;
        if (missed_old || new_source)
            channel.requests.insert(
                std::upper_bound(channel.requests.begin(), channel.requests.end(), time), time);
    }
    channel.early.clear();

    // Model-defined: the AES requests (ENC_DW/ENC_UP, SWRU191F 15.9 "active
    // when input data must be downloaded" / "output data must be uploaded")
    // are levels; a channel that becomes ready while its AES request is
    // active sees one trigger. This makes the DN108 sequence (arm, NOP,
    // ENCCS.ST within the 9-clock fetch) work.
    if (channel.requests.empty() && (channel.trig == trigger::EncDw || channel.trig == trigger::EncUp) &&
        bus_.soc().aes->dma_level(channel.trig))
        channel.requests.push_back(at);

    // SWRU191F 8.5: the DMAIRQ flag is set regardless of IRQMASK; "software
    // should always check (and clear) this register when rearming a channel
    // with a changed IRQMASK setting. Failure to do so could generate an
    // interrupt based on the stored interrupt flag."
    if (channel.irqmask && !old_irqmask &&
        (bus_.storage(sfr_reg(SfrDmairq)) & (1u << index)))
        bus_.raise_irq(Irq::Dma);
}

unsigned Dma::slot_cycles(const Channel &channel) {
    return channel.word ? WordSlotCycles : ByteSlotCycles;
}

std::uint64_t Dma::eligible_at(unsigned index) const {
    const Channel &channel = channels_[index];
    if (channel.state != State::Armed)
        return Never;
    if (channel.in_block)
        return channel.armed_at; // a started block continues without triggers
    std::uint64_t at = Never;
    if (bus_.storage(sfr_reg(SfrDmareq)) & (1u << index))
        at = channel.dmareq_at;
    if (!channel.requests.empty())
        at = std::min(at, channel.requests.front());
    return at == Never ? Never : std::max(at, channel.armed_at);
}

void Dma::set_paused(bool paused) {
    if (paused == paused_)
        return;
    advance(bus_.hf_now());
    paused_ = paused;
    bus_.reschedule();
}

std::uint64_t Dma::arbitration_time() const {
    if (slot_active_ || paused_)
        return Never;
    std::uint64_t at = Never;
    for (unsigned index = 0; index < Channels; ++index)
        at = std::min(at, eligible_at(index));
    if (at == Never)
        return Never;
    return std::max({at, engine_free_, in_service_ ? cursor_ : watermark_});
}

std::uint64_t Dma::own_next_event() const {
    std::uint64_t at = slot_active_ ? slot_end_ : Never;
    for (const Channel &channel : channels_)
        if (channel.state == State::Configuring)
            at = std::min(at, channel.ready_at);
    return std::min(at, arbitration_time());
}

void Dma::advance(std::uint64_t limit) {
    if (in_service_)
        return;
    in_service_ = true;
    cursor_ = std::max(cursor_, watermark_);
    FlashController &flash = *bus_.soc().flash;
    Aes &aes = *bus_.soc().aes;
    for (;;) {
        // Model-defined order of events at the same clock: flash controller,
        // AES, end of a data slot, end of a descriptor fetch, arbitration.
        const std::uint64_t flash_at = flash.next_event();
        const std::uint64_t aes_at = aes.next_event();
        const std::uint64_t slot_at = slot_active_ ? slot_end_ : Never;
        std::uint64_t ready_at = Never;
        unsigned ready_index = 0;
        for (unsigned index = 0; index < Channels; ++index)
            if (channels_[index].state == State::Configuring && channels_[index].ready_at < ready_at) {
                ready_at = channels_[index].ready_at;
                ready_index = index;
            }
        const std::uint64_t arbitrate_at = arbitration_time();
        const std::uint64_t at =
            std::min({flash_at, aes_at, slot_at, ready_at, arbitrate_at});
        if (at == Never || at > limit)
            break;
        cursor_ = std::max(cursor_, at);
        if (flash_at == at)
            flash.process_until(at);
        else if (aes_at == at)
            aes.process_until(at);
        else if (slot_at == at)
            finish_slot(at);
        else if (ready_at == at)
            load_descriptor(ready_index, at);
        else
            arbitrate(at);
    }
    watermark_ = std::max(watermark_, limit);
    cursor_ = watermark_;
    in_service_ = false;
}

void Dma::arbitrate(std::uint64_t at) {
    // SWRU191F 8.2.8: the highest DMA priority wins; "in case of an internal
    // tie, a round-robin scheme is used". Model-defined: the round-robin
    // starts after the channel granted last (initially channel 0 first);
    // priority with respect to the CPU is not modeled (the CPU never waits).
    int winner = -1;
    unsigned winner_rank = 0;
    for (unsigned offset = 1; offset <= Channels; ++offset) {
        const unsigned index = (last_granted_ + offset) % Channels;
        const Channel &channel = channels_[index];
        if (eligible_at(index) > at)
            continue;
        const unsigned rank = priority_rank(channel.priority);
        if (winner < 0 || rank > winner_rank) {
            winner = static_cast<int>(index);
            winner_rank = rank;
        }
    }
    if (winner < 0)
        return;
    const auto index = static_cast<unsigned>(winner);
    Channel &channel = channels_[index];
    if (!channel.in_block) {
        // Consume the request that starts this transfer (block: the whole
        // block). The DMAREQ bit "is cleared when DMA transfer is started".
        // Model-defined: a pending DMAREQ is consumed before counted
        // hardware triggers.
        std::uint8_t &dmareq = bus_.storage(sfr_reg(SfrDmareq));
        if ((dmareq & (1u << index)) && channel.dmareq_at <= at)
            dmareq = static_cast<std::uint8_t>(dmareq & ~(1u << index));
        else if (!channel.requests.empty())
            channel.requests.erase(channel.requests.begin());
        if (block_mode(channel.tmode))
            channel.in_block = true;
    }
    slot_active_ = true;
    slot_channel_ = index;
    slot_generation_ = channel.generation;
    slot_end_ = at + slot_cycles(channel);
    last_granted_ = index;
}

void Dma::finish_slot(std::uint64_t at) {
    slot_active_ = false;
    engine_free_ = at;
    const unsigned index = slot_channel_;
    Channel &channel = channels_[index];
    const std::uint32_t generation = slot_generation_;
    if (channel.generation != generation || channel.state != State::Armed)
        return;
    if (channel.len == 0) {
        // Model-defined: LEN = 0 completes the channel at its first transfer
        // slot without moving data.
        complete(index, at);
        return;
    }
    const std::uint8_t low = bus_.xdata_read(channel.src_ptr);
    const std::uint8_t high =
        channel.word ? bus_.xdata_read(static_cast<std::uint16_t>(channel.src_ptr + 1)) : 0;
    if (channel.generation != generation)
        return;
    if (!channel.total_known) {
        // SWRU191F 8.2.4 / Table 8-2 VLEN: the first byte or word (bits 12:0)
        // is the length n; M8 uses its 7 LSBs (byte mode only). LEN is the
        // maximum transfer count. VLEN 001: n + 1 transfers (the length plus
        // n bytes), 010: n (including the length), 011: n + 2, 100: n + 3.
        // Model-defined: a word's low byte is the one at the lower address,
        // and at least the length itself is transferred.
        std::uint32_t n = channel.word ? ((static_cast<std::uint32_t>(high) << 8 | low) & 0x1fffu)
                                       : (channel.m8 ? low & 0x7fu : low);
        static constexpr std::uint32_t Extra[5] = {0, 1, 0, 2, 3};
        n += Extra[channel.vlen];
        channel.total = std::max<std::uint32_t>(1, std::min<std::uint32_t>(n, channel.len));
        channel.total_known = true;
    }
    bus_.xdata_write(channel.dst_ptr, low);
    if (channel.word && channel.generation == generation)
        bus_.xdata_write(static_cast<std::uint16_t>(channel.dst_ptr + 1), high);
    if (channel.generation != generation)
        return; // the write aborted or rearmed this channel
    // Addresses wrap at 16 bits.
    channel.src_ptr = static_cast<std::uint16_t>(channel.src_ptr + step(channel.srcinc, channel.word));
    channel.dst_ptr =
        static_cast<std::uint16_t>(channel.dst_ptr + step(channel.destinc, channel.word));
    if (++channel.done >= channel.total)
        complete(index, at);
}

void Dma::complete(unsigned index, std::uint64_t at) {
    Channel &channel = channels_[index];
    // Figure 8-1 / 8.5: DMAIRQ.DMAIFn is set on completion regardless of
    // IRQMASK; IRCON.DMAIF only if IRQMASK = 1.
    std::uint8_t &dmairq = bus_.storage(sfr_reg(SfrDmairq));
    dmairq = static_cast<std::uint8_t>(dmairq | (1u << index));
    if (channel.irqmask)
        bus_.raise_irq(Irq::Dma);
    channel.in_block = false;
    channel.done = 0;
    channel.total_known = false;
    if (repeated_mode(channel.tmode)) {
        // 8.2.7: repeated modes rearm (reload the configuration, Figure 8-1);
        // counted triggers stay pending.
        start_fetch(static_cast<std::uint8_t>(1u << index), at);
    } else {
        // 8.8 DMAARMn: "For nonrepetitive transfer modes, the bit is
        // automatically cleared on completion."
        channel.state = State::Idle;
        channel.requests.clear();
        ++channel.generation;
        set_arm_bit(index, false);
    }
    // Table 8-1 trigger 1 (PREV): "DMA channel is triggered by completion of
    // previous channel" - channel n completing triggers channel n + 1.
    if (index + 1 < Channels)
        record(index + 1, trigger::Prev, at);
}

void Dma::abort(std::uint8_t channels, std::uint64_t at) {
    // SWRU191F 8.4: writing ABORT together with DMAARMn = 1 aborts (disarms)
    // those channels; no DMAIRQ flag and no interrupt. DMAREQ bits are not
    // cleared by disarming (8.1). Model-defined: an aborted in-flight data
    // slot writes nothing and frees the engine at once; counted triggers of
    // the aborted channel are discarded.
    for (unsigned index = 0; index < Channels; ++index) {
        if (!(channels & (1u << index)))
            continue;
        Channel &channel = channels_[index];
        channel.state = State::Idle;
        channel.in_block = false;
        channel.done = 0;
        channel.total_known = false;
        channel.requests.clear();
        channel.early.clear();
        ++channel.generation;
        set_arm_bit(index, false);
        if (slot_active_ && slot_channel_ == index) {
            slot_active_ = false;
            engine_free_ = at;
        }
    }
}

} // namespace mcs51emu::cc2530
