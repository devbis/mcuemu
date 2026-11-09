/* Software 32-bit division. The SDK's platform/tc32/div_mod.S drives the B85's
 * hardware divider (0x660..); the 826x register map has no such block, so the
 * smoke firmware supplies the compiler helpers itself. */
unsigned int __udivsi3(unsigned int n, unsigned int d) {
    unsigned int q = 0, r = 0;
    for (int i = 31; i >= 0; --i) {
        r = (r << 1) | ((n >> i) & 1u);
        if (r >= d) { r -= d; q |= 1u << i; }
    }
    return q;
}
unsigned int __umodsi3(unsigned int n, unsigned int d) { return n - __udivsi3(n, d) * d; }
int __divsi3(int n, int d) {
    int neg = (n < 0) ^ (d < 0);
    unsigned int q = __udivsi3(n < 0 ? -(unsigned int)n : (unsigned int)n,
                               d < 0 ? -(unsigned int)d : (unsigned int)d);
    return neg ? -(int)q : (int)q;
}
int __modsi3(int n, int d) { return n - __divsi3(n, d) * d; }
