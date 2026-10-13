# План архитектуры: декларативное описание устройств/сетей (YAML) для mcuemu

## Context

Сейчас эмулятор (C++20, CMake, Unicorn-форк) собирает устройства хардкодом:
- `Tlsr8258` — монолит: Bus + flash/SRAM + все периферии как ветки в `read_register/write_register`; `Cpu` жёстко привязан к `Tlsr8258&`.
- `Ca51F253L3` — самодостаточный класс, с `Tlsr8258` общего кода нет.
- Единственная абстракция — `Bus` (`bus.hpp`); единственная уже развязанная с чипом часть — `VirtualAir`/`VirtualRadioNode` (`radio.hpp`).
- Конфигурация только CLI-флагами (`network_command` в `emulator/src/main.cpp`: `--node NAME=bin`, `--channel`, `--start-at`, `--gpio-event`). Скриптинга нет, внешних устройств (датчики, второй MCU) нет.
- Время: TC32 — константа 3 цикла/инструкцию; mcs51 — реальный счётчик циклов; мульти-нода — round-robin по 256 инструкций с общим `global_time_ns` (не очередь событий).

Цель: YAML описывает **систему** = набор устройств = MCU + внешняя периферия + проводка между ними + радио-среда, плюс скриптуемые модели периферии (например DS18B20).

## Целевая модель (общая для всех вариантов)

Слои:
1. **Machine/Board** — YAML: `world → devices → mcus + parts + nets`.
2. **MCU-модели** (`tlsr8258`, `ca51f253l3`, …) — реестр фабрик `McuFactory(name) → Mcu`; общий интерфейс `Mcu`: `load_firmware`, `run_until(t_ns)`, `pin(i)`, `bus(kind)` (uart/spi/i2c endpoint'ы), `radio()`.
3. **Pin/Net-слой** — центр всей проводки (см. ниже).
4. **Parts** (внешняя периферия) — реестр фабрик, нативные (C++) или скриптовые.
5. **Scheduler** — глобальная очередь событий по виртуальному времени.
6. **Air** — расширенный `VirtualAir` (несколько сред/каналов, протоколы).
7. **Scripting host** — встраиваемый язык для parts и сценариев теста.

### Пины и сети (ключевое решение)
- Каждый вывод MCU — `Pin` с состоянием `{drive: hi/lo/hiz/pullup/pulldown, level, analog_mV}`.
- `Net` — именованный узел, объединяющий пины (open-drain/wired-AND для I2C и 1-Wire, push-pull для остальных). Разрешение уровня: по приоритетам драйверов, с детектированием конфликтов (warning).
- Подписка на изменения уровня → события в scheduler с временной меткой (нужно для 1-Wire и bit-bang I2C, где важны µs).
- Аналог: ADC читает `analog_mV` нета; NTC = part, которая выставляет напряжение делителя на нете (с учётом pullup MCU, если нужно — простая модель резисторного делителя).
- Шины высокого уровня (UART/SPI/I2C между двумя MCU или MCU↔sensor) — **две реализации**: (a) через пины/Net бит-точно (медленнее, универсально, работает и с bit-bang), (b) «shortcut» `link:` на уровне байтов/транзакций между периферийными блоками (быстро; включается, когда обе стороны — аппаратные контроллеры). Начать с (a) для 1-Wire/I2C и с (b) для UART.

### Время
- Единый `Scheduler` с `time_ns` и очередью событий (`at(t, fn)`); MCU исполняется квантами до ближайшего события (`run_until`), а не по 256 инструкций. Это заменяет round-robin из `network_command`, делает UART/SPI/I2C между MCU и радио детерминированными.
- Для TC32 заменить константу 3 цикла на таблицу стоимостей инструкций (можно позже; на архитектуру не влияет, но влияет на точность 1-Wire/bit-bang).
- Жёсткое правило: детерминизм (один seed RNG на world, никаких wall-clock).

## Схема YAML (черновик)

```yaml
version: 1
world:
  seed: 1
  radio:
    - id: air0
      model: ideal            # ideal | pathloss
      # positions/pathloss — для варианта с RSSI
devices:
  - id: sensor_node
    mcus:
      - id: main
        model: tlsr8258       # реестр
        variant: F512
        firmware: fw/sensor.bin
        clock_hz: 48000000
        start_at: 0ms
        radio: { air: air0, protocols: [zigbee, ble] }   # coexistence — оба
      - id: aux
        model: ca51f253l3
        firmware: fw/aux.bin
    nets:
      - id: ow
        kind: open_drain
        pull_up: { ohm: 4700, to: vdd }
        pins: [main.PB4]
      - id: i2c_sda   { kind: open_drain, pins: [main.PC0, aux.P3_1, th_sensor.sda] }
      - id: uart_ab   # или shortcut:
    links:
      - { type: uart, a: main.uart0, b: aux.uart1, baud: 115200 }   # shortcut-вариант
    parts:
      - id: temp1
        type: ds18b20            # нативный или скриптовый
        pins: { dq: ow }
        params: { rom: "28-AA-...", temperature_c: 23.5 }
      - id: ntc1
        type: ntc
        pins: { out: main.PB5 }  # аналоговый
        params: { r25: 10k, beta: 3950, r_pullup: 10k, temp_c: 25 }
      - id: th
        type: script
        script: parts/sht31.lua
        pins: { sda: i2c_sda, scl: i2c_scl }
  - id: ble_only        # устройство-одиночка
    mcus: [ { id: m, model: tlsr8258, firmware: fw/ble.bin, radio: {air: air0, protocols: [ble]} } ]
  - id: zb_coordinator
    mcus: [ { id: m, model: tlsr8258, firmware: fw/coord.bin, radio: {air: air0, protocols: [zigbee]} } ]
scenario:               # опционально: скриптовые события
  script: scenarios/join.lua
  events:
    - { at: 2s, target: sensor_node.temp1, set: { temperature_c: 40 } }
    - { at: 5s, target: sensor_node.main.PA1, drive: low }
```

Валидация: JSON Schema + семантические проверки (все ссылки `device.mcu.pin` существуют, пин не в двух конфликтующих ролях, модель MCU знает пин, `part` знает свои порты). Ошибки с указанием строки YAML.

## Варианты реализации

### Вариант A — Ядро на C++, YAML в C++, скрипты через встроенный Lua (рекомендуется)
- **Парсинг**: `yaml-cpp` (FetchContent, как и сейчас Unicorn; либо `ryml`). Загрузка → `SystemConfig` (plain structs) → `SystemBuilder` с реестрами `McuRegistry`/`PartRegistry`.
- **Скрипты**: Lua 5.4 (+ `sol2` или чистый C API). Минимальная стоимость интеграции, легко встраивается, детерминирован, без внешних зависимостей рантайма, быстрый.
- **Plus**: один бинарь `mcuemu sim system.yaml`, высокая скорость, без Python-зависимости в CI.
- **Minus**: Lua не знаком большинству; нужно написать/зафиксировать API.

### Вариант B — Ядро C++ + Python как хост (pybind11/nanobind), YAML — тонкая обёртка
- Ядро выставляет `World`, `Net`, `Part`, `Scheduler` в Python; YAML читается Python-кодом (PyYAML) и собирает объекты; parts пишутся на Python-классах.
- **Plus**: тесты уже на Python (`tests/firmware/*.py`), максимальная гибкость сценариев, экосистема.
- **Minus**: зависимость от Python для любого запуска; GIL/скорость на обратных вызовах per-pin-edge (1-Wire — тысячи событий); сложнее детерминизм; сложнее поставка CLI.
- Подходит, если основной потребитель — разработчики тестов.

### Вариант C — Гибрид (A + Python-API поверх)
- Ядро и `mcuemu sim` как в A (Lua для part-скриптов — быстрый путь), плюс опциональный `nanobind`-модуль для оркестрации тестов (запустить world, дернуть `world.at(2s).set(...)`, прочитать лог). Part-скрипты на Python — только «медленные» (сенсоры, не бит-банг).
- **Plus**: лучшее из двух; **Minus**: два API для поддержки. Сделать после A, когда API стабилизируется.

### Варианты скриптинга parts (внутри A/C)
| Подход | Для чего |
|---|---|
| Нативный C++ part (`Ds18b20`, `Ntc`, `GpioButton`, `I2cEeprom`) | быстрые, критичные по времени (1-Wire) |
| Lua part: callbacks `on_pin_edge(net, t, level)`, `on_i2c_start/byte`, `on_timer`; API `net:drive(level)`, `sched:at(dt, fn)`, `now()` | быстрое прототипирование, пользовательские датчики |
| Декларативный FSM в YAML (states/transitions) | простейшие устройства (кнопка, реле, LED-лог) без кода |
| WASM-плагины | отвергнуто пока (избыточно) |

Рекомендация: нативный DS18B20 как референс + Lua-API, достаточный чтобы переписать его на Lua (критерий приёмки API: DS18B20 на Lua проходит тот же тест, что и нативный).

## Наблюдаемость и управление: Endpoints + Control API

### Принцип: один механизм, три способа управления
Всё, что можно «посмотреть/потрогать», — это **Endpoint** (именованная точка в системе). Подключение к ней — **Attachment** (подписка/привязка потребителя). Конфиг YAML, консоль CLI и клиентский API — это три способа создать/удалить Attachment через **один и тот же** `Session`-интерфейс. Поэтому вопрос «конфиг или override через консоль» снимается: делаем слоями, приоритет по возрастанию:
1. YAML `attach:` — значения по умолчанию, воспроизводимый сценарий;
2. CLI-флаги (`--attach uart:sensor_node.main.uart0=tcp:5000`) — переопределяют/добавляют, разворачиваются в те же вызовы API;
3. Runtime (интерактивная консоль/REPL, WebSocket/gRPC) — меняет на лету; не пишется в YAML (опционально `save` дампит текущее состояние в YAML).
Ограничение: структура системы (устройства, проводка, прошивки) — только из YAML; attach/детач, значения воздействий — в любом слое.

### Каталог Endpoint'ов (адресация `device.mcu.name` / `device.part.name`)
| Тип | Направление | Источник данных |
|---|---|---|
| `uart` | in/out байты | UART-контроллер MCU (`take_uart_transmitted`/`inject_uart_rx` уже есть в TLSR8258) |
| `rtt` | in/out байты по каналам | SEGGER RTT: найти `_SEGGER_RTT` control block по символу/сканом SRAM, читать ring buffer «up» и писать «down» прямо из памяти Unicorn без вмешательства в CPU (нужна карта символов/адрес в YAML: `rtt: { symbol: _SEGGER_RTT }` или `addr:`) |
| `pin` | out события/уровень; in drive | `Pin`/`Net` (уровень, drive, analog_mV, лог переходов с timestamp) |
| `led` / `button` / `relay` | out / in | parts, привязанные к пинам (`led` = пин + полярность, + опциональный PWM duty из `output_events()`) |
| `part.<param>` | get/set | параметры part (температура DS18B20, R NTC) |
| `radio` | out кадры | Medium: журнал кадров (канал, протокол, t_start/end, payload, RSSI), pcap-экспорт (802.15.4 / BLE) |
| `memory` / `reg` | get/set | чтение/запись SRAM/MMIO/регистров CPU (отладка) |
| `trace` | out | существующие trace (PC, MMIO, stack) как поток |
| `time` | control | пауза/шаг/скорость/`run_for`/`run_until` |

### Транспорты Attachment (приёмники)
- `stdout`/`file` (лог с timestamp виртуального времени), `tcp`/`pty`/`unix` socket (подключить терминал/minicom/`nc` к UART или RTT, как RTT Telnet-порт J-Link), `ws` (стрим в браузер), `callback` (в Lua/Python), `pcap`, `vcd` (пины → осциллограф-вид в GTKWave), `memory ring` (для опроса клиентом).
- Для in-направления: тот же канал принимает байты из TCP → инжектит в UART/RTT.

### Фрагмент YAML
```yaml
attach:
  - { endpoint: sensor_node.main.uart0, sink: tcp:5000 }          # терминал к UART
  - { endpoint: sensor_node.main.rtt,   channel: 0, sink: file:rtt.log }
  - { endpoint: sensor_node.status_led, sink: stdout, on: change } # «лампочка»
  - { endpoint: sensor_node.main.PB4,   sink: vcd:pins.vcd }
  - { endpoint: air0, sink: pcap:air.pcap }
```

### Control API (клиентское API для веб-сервиса)
Ядро получает библиотечный слой `Session`/`Simulation` (без CLI-зависимостей; CLI `sim` — лишь один из клиентов). Поверх него **сервер** `mcuemu serve` (отдельный процесс/режим) с протоколом, не привязанным к языку:

**Вариант 1 (рекомендуется): gRPC + WebSocket/SSE-шлюз или чистый JSON-RPC 2.0 поверх WebSocket.**
- Команды (request/response): `load(yaml)`, `start/pause/step/run_for(dur)/set_speed(x)`, `list_endpoints()`, `get(endpoint)`, `set(endpoint, value)`, `attach(endpoint, sink)`, `detach(id)`, `uart.write(ep, bytes)`, `mem.read/write`, `snapshot/restore` (позже), `shutdown`.
- События (server push): подписка `subscribe(endpoint[, filter])` → поток `{t_virtual_ns, endpoint, kind, data}` (байты UART/RTT, смена пина/LED, радио-кадры, остановка по breakpoint/ошибке).
- Для веб-UI: браузер ↔ WebSocket JSON напрямую (простота; xterm.js для UART/RTT, SVG-лампочки/пины, график температуры); для бэкенда — gRPC/Protobuf (типизация, стриминг). Схема сообщений описывается один раз (protobuf или JSON Schema → генерация TS/Python клиентов).
- Многосессионность: сервер хранит `Session` по id (изоляция, лимиты ресурсов, таймаут простоя); одна сессия = один `World`.
- Безопасность: по умолчанию bind на localhost; токен-аутентификация; пути к файлам в `load`/sink'ах — только в разрешённых каталогах (sandbox), `tcp:` sink-порты — из пула.

**Вариант 2: только REST + SSE.** Проще, но неудобно для двунаправленных потоков (UART in/out); отвергнуто в пользу WS/gRPC.
**Вариант 3: Python-bindings in-process** (см. вариант C) — для автотестов; для веб-сервиса используется поверх Вариант 1 или внутри FastAPI-приложения. Не заменяет серверный протокол.

### Модель времени для интерактива
- `realtime: false` (по умолчанию, максимальная скорость, детерминированно) vs `realtime: x1.0` (виртуальное время привязано к wall-clock — нужно для интерактивного UI/терминала). Реализуется throttling'ом в цикле Scheduler, внешний ввод (UART in, нажатие кнопки) помечается виртуальным временем «сейчас» и вставляется в очередь — детерминизм воспроизводим через запись входных событий (`record`/`replay` лог).
- Внешние события всегда проходят через Scheduler (не прямой вызов из сетевого потока): сетевой поток кладёт команды в потокобезопасную очередь, sim-поток применяет на границе кванта.

### Что это меняет в архитектуре
- Добавить слой **Endpoint registry** (`EndpointRegistry`: имя → объект с `read/write/subscribe`), каждый MCU/part/net/medium регистрирует свои endpoints при построении (`SystemBuilder`). `list_endpoints()` отдаёт дерево для UI.
- Добавить `Sink`-фабрики (tcp/ws/file/pcap/vcd/stdout/callback).
- Sim работает в отдельном потоке; один владелец состояния (no locks на hot path), коммуникация — очередь команд + подписки (лок-фри кольцо для стримов).
- Существующие `--trace-*`, `--dump-*`, `--radio-payloads`, `--gpio-event` переписываются как алиасы на Attachments/Scenario events.

## Радио-пространство

- Расширить `VirtualAir` до `Medium`: узлы регистрируются с `{protocols, channel(s), position?, tx_power}`.
- Coexistence-устройство: один MCU владеет несколькими `RadioEndpoint` (zigbee и ble), которые делят одну радио-железку: арбитраж на уровне TLSR8258-модели (в какой момент она в TX/RX какого протокола), `Medium` видит физические передачи с частотой/каналом/временем, а не «протокол».
- Совместимость каналов: BLE (2 МГц, 40 каналов) и 802.15.4 (5 МГц, ch11–26) мапятся в общую частотную ось → корректно считаются перекрытия/интерференция между BLE и Zigbee на одной частоте.
- Модели среды: `ideal` (как сейчас, broadcast в канале) → `pathloss` (позиции, RSSI, порог чувствительности, SINR-коллизии) — вводить поэтапно.
- BLE пока не подключён к регистрам TLSR8258 (по docs/architecture.md) — отдельная под-задача, блокирует BLE-сценарии, но не архитектуру.

## Рефакторинг, необходимый для архитектуры

1. Ввести интерфейс `Mcu` (над `Cpu`+chip) и отвязать `Cpu` от `Tlsr8258&` (использовать `Bus`/`ChipHooks`). Мигрировать mcs51 на тот же интерфейс.
2. Разбить `Tlsr8258` на периферийные блоки с регистрацией диапазонов адресов (`MmioDevice{range, read, write, tick, irq}`); начать с тех, что выходят наружу: GPIO, UART, SPI, I2C, ADC, radio. Остальное можно оставить внутри.
3. Вынести GPIO в `Pin`/`Net`: заменить `set_gpio_input/gpio_output/output_events` на подписки Net.
4. Scheduler + `run_until(t)` у обоих MCU.
5. Реестры и билдер; существующие CLI `run`/`network` реализовать поверх сгенерированной YAML-конфигурации (обратная совместимость).

## Этапы

**Статус:** этап 1 выполнен (`docs/architecture.md`, раздел «Simulation core»); этап 3a выполнен (parts: ds18b20, ntc, i2c_regs, `I2cSlave`; события `endpoint`+`set`). этап 3b выполнен: ADC (misc-канал), аппаратный I2C-мастер и подтяжки GPIO в `Tlsr8258` (SPI не сделан); этап 4 выполнен (Lua-хост `LuaPart`, скриптовый DS18B20 проходит те же тесты, что и нативный; `docs/scripting.md`); этап 5 выполнен (`VirtualAir`: позиции, потери, RSSI, SINR-захват, общая частотная ось BLE/802.15.4, полудуплекс; YAML `world.radio` и `radio:` у mcu/part; Lua `radio.*`). Не сделано: BLE в регистрах TLSR8258 (значит coexistence-MCU с двумя протоколами пока невозможен, BLE-трафик идёт от Lua-устройств); этап 6 выполнен (`Session`/`Sink`: stdout, file, tcp, vcd, pcap; endpoints uart/rtt; `attach:` в YAML и `--attach`; parts led/button). Не сделано: `memory`/`reg`/`trace`/`time` endpoints (нужны для Control API); этап 7 выполнен (`mcuemu serve`: JSON-RPC 2.0 поверх WebSocket, `docs/control-api.md`; песочница `--root`, токен, Origin, лимиты). этап 7+: realtime-пейсинг (`set_speed`/`run.speed`, верхняя граница скорости, без догоняющих всплесков). Не сделано: TLS, endpoints `memory`/`reg`/`trace` как потоки; этап 2 выполнен (формат: `docs/system-config.md`; в реализации сети задаются как `pull: up|down` вместо `pull_up: {ohm,to}`, который тоже принимается, сопротивление пока не моделируется).

1. **Ядро**: `Mcu` интерфейс, `Scheduler`, `Pin/Net`; GPIO TLSR8258 через Net; mcs51 через тот же интерфейс. Тесты: два MCU, кнопка → GPIO.
2. **YAML + билдер + схема**: `sim system.yaml`; одно- и двухчиповые устройства; `link: uart` между MCU.
3. **Parts**: NTC (ADC), DS18B20 (нативный, 1-Wire), I2C-датчик; I2C/SPI-контроллеры в TLSR8258 с Net/link.
4. **Скриптинг**: Lua-хост, API parts, `scenario.events`; DS18B20 на Lua = эталонный тест.
5. **Радио**: `Medium` с позициями/pathloss/RSSI/SINR сразу, несколько устройств (coexistence + ble + coordinator), общая частотная ось; `ideal` — частный случай.
6. **Endpoints/Attachments**: `EndpointRegistry`, sinks (stdout/file/tcp), UART и pin-endpoints, `attach:` в YAML + `--attach` в CLI; затем RTT (чтение ring buffer из памяти), led/button, pcap/vcd.
7. **Control API**: `Session` библиотека + `serve` (JSON-RPC 2.0 over WebSocket), `subscribe`-стримы, `pause/step/run_for`; без realtime-throttling.
8. **Python**: `PythonHost` для parts/сценариев (тот же API, что у Lua), Python-клиент Control API; затем snapshot/restore (опц.).

Примечание: слой `Session`/Endpoint закладывать уже на этапе 1 (интерфейсы), даже если реализации появятся позже — иначе CLI-флаги придётся переписывать дважды.

## Критичные файлы
- `emulator/include/mcuemu/{bus,cpu,tlsr8258,radio}.hpp`, `emulator/src/{cpu,tlsr8258,radio,main}.cpp`
- `emulator/include/mcs51emu/ca51f253l3.hpp`, `emulator/src/mcs51/*`
- `CMakeLists.txt` (новые зависимости: yaml-cpp, Lua), новый каталог `emulator/src/sim/` (config, builder, scheduler, net, parts, script), `docs/architecture.md` (обновить)

## Верификация
- Unit-тесты (существующий `tests/test_support.hpp`): Net-резолвер (open-drain, конфликты), Scheduler-порядок, валидатор YAML (ошибочные ссылки).
- Интеграционные: YAML 2×MCU по UART обменивается байтами; DS18B20 (нативный и Lua) читается прошивкой по 1-Wire; NTC даёт ожидаемый ADC-код; 3 устройства в одном `air0` — join Zigbee (текущий `test_zigbee_join.py` без правок через новый YAML).
- Регрессия: `make test`, существующие `run`/`network` команды дают те же результаты.

## Принятые решения (по ответам пользователя)
- **Скриптинг**: сначала Lua (вариант A), затем Python (вариант C) — потому что Lua мало кто знает. Чтобы Python не пришлось переделывать: `Part`-интерфейс и событийное API (`on_pin_edge`, `on_i2c_byte`, `sched.at`, `net.drive`, `now`) определяются на C++ один раз, а Lua и Python — два равноправных биндинга к нему (`ScriptHost`-интерфейс с реализациями `LuaHost`, позже `PythonHost` через nanobind/embed). YAML `type: script` выбирает язык по расширению (`.lua`/`.py`). Python также используется как клиент Control API.
- **RSSI/pathloss — закладываем сразу**: `Medium` с позициями узлов (`position: [x,y,z]` в YAML), `tx_power_dbm`, моделью потерь (`pathloss: {model: log_distance, n:, ref_loss_db:}` + опц. shadowing/seed), чувствительностью приёмника и SINR-коллизиями; RSSI/LQI передаётся в RX-метаданные (в регистры/DMA-заголовок TLSR8258). Режим `ideal` остаётся как частный случай (потери 0, без позиций) для обратной совместимости.
- **Протокол сервера**: JSON-RPC 2.0 поверх WebSocket (gRPC не делаем).
- **RTT**: пока только по адресу (`rtt: {addr: 0x...}` в YAML/attach), без ELF-символов и скана.
- **Realtime**: делаем как проще — только максимальная скорость + `pause/step/run_for`; throttling под wall-clock не реализуем в v1 (запись/воспроизведение входов остаётся опционально).

## Открытые вопросы
- Достаточна ли точность по времени инструкций TC32 (константа 3) для bit-bang 1-Wire? Проверить на этапе 3 (DS18B20); при необходимости ввести таблицу циклов.
