////////////////////////////////////////////////////////////
//
//   FilterFuzzy.h
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Circuit simulation engine for the Fuzzy models, by the nodal DK method.
//
//   A circuit (or one stage of a circuit) is a netlist: resistors,
//   capacitors, inductors, current sources driven by the input vector u,
//   bipolar transistors (PNP or NPN, Ebers-Moll in injection form) and
//   diodes. buildDK() reduces it to the state-space form the audio loop
//   runs; the same matrices at a huge backward-Euler step give the DC
//   operating point. Junctions are solved by Newton with the exponentials
//   in float_4 and everything else in double for required precision.
//
//   Circuits are described in schematic terms: node voltages are measured
//   from the schematic ground, which is also the jack sleeve and the signal
//   reference, and the battery drives a rail node (negative for the PNP
//   germanium circuits, positive for the NPN silicon ones).
//
////////////////////////////////////////////////////////////

#pragma once
#include "rack.hpp"
#include <cmath>
#include <cstring>
#include <cstdint>
#include <algorithm>

namespace fuzzy {

using rack::simd::float_4;

static const int GND = -1;

// The u vector, shared layout for every stage:
//   u[0]          the signal: source EMF, or the voltage handed on by the
//                 previous stage. The only entry that moves within a sample.
//   u[1]          the battery voltage (signed: -9 for a PNP rail)
//   u[2 .. I-2]   collector-base leakage of each germanium transistor, amps
//   u[I-1]        a noise current, amps
static const int INPUT_SIGNAL = 0;
static const int INPUT_RAIL   = 1;
static const int INPUT_LEAK   = 2;

inline double thermalVoltage(double temperatureC) {
    return 1.380649e-23 * (temperatureC + 273.15) / 1.602176634e-19;
}

// ==========================================================================
// Netlist
// ==========================================================================
enum ElementType { EL_RES, EL_CAP, EL_IND, EL_ISRC, EL_BJT, EL_DIODE };

struct Element {
    ElementType type;
    int    a, b;              // two-terminal elements; current flows a -> b
    double value;             // ohms, farads, henries; ISRC: gain from u
    int    input;             // ISRC: which u entry drives it (current INTO a)
    int    emitter, base, collector;
    double polarity;          // BJT: +1 PNP, -1 NPN
    double alphaF, alphaR;
    int    port;              // BJT: forward port (reverse is port + 1); DIODE: its port
};

static const int MAX_ELEMENTS = 48;

struct Netlist {
    Element el[MAX_ELEMENTS];
    int count = 0;

    Element& add(ElementType type) {
        Element& e = el[count < MAX_ELEMENTS ? count++ : MAX_ELEMENTS - 1];
        e = Element();
        e.type = type;
        e.a = e.b = e.emitter = e.base = e.collector = GND;
        e.input = e.port = -1;
        return e;
    }
    // Floors keep a pot turned to its end from making a singular network.
    void res(int a, int b, double ohms)    { Element& e = add(EL_RES); e.a = a; e.b = b; e.value = std::fmax(ohms, 1.0); }
    void cap(int a, int b, double farads)  { Element& e = add(EL_CAP); e.a = a; e.b = b; e.value = std::fmax(farads, 1e-15); }
    void ind(int a, int b, double henries) { Element& e = add(EL_IND); e.a = a; e.b = b; e.value = henries; }
    // Current gain * u[input] flowing INTO node.
    void isrc(int node, double gain, int input) { Element& e = add(EL_ISRC); e.a = node; e.value = gain; e.input = input; }
    void bjt(int emitter, int base, int collector, bool pnp, double betaF, double betaR, int port) {
        Element& e = add(EL_BJT);
        e.emitter = emitter; e.base = base; e.collector = collector;
        e.polarity = pnp ? 1.0 : -1.0;
        e.alphaF = betaF / (betaF + 1.0);
        e.alphaR = betaR / (betaR + 1.0);
        e.port = port;
    }
    void diode(int anode, int cathode, int port) { Element& e = add(EL_DIODE); e.a = anode; e.b = cathode; e.port = port; }
};

// ==========================================================================
// Junctions. Every port is one exponential of its own voltage:
//   i = Is * (exp(v / (n Vt)) - 1)
// BJT, injection-form Ebers-Moll: ports vEB and vCB for a PNP (vBE and vBC
// for an NPN), with IsF = Is / alphaF and IsR = Is / alphaR. The coupling
// between them is linear and lives in the injection columns, so di/dv is
// diagonal: one exp per junction and no cross terms.
// ==========================================================================
template <int P>
struct PortModel {
    double saturation[P];
    double invNVt[P];

    void setTransistor(int port, double is, double emission, double betaF, double betaR, double temperatureC) {
        double invNVtValue = 1.0 / (emission * thermalVoltage(temperatureC));
        saturation[port]     = is * (betaF + 1.0) / betaF;
        saturation[port + 1] = is * (betaR + 1.0) / betaR;
        invNVt[port] = invNVt[port + 1] = invNVtValue;
    }
    void setDiode(int port, double is, double emission, double temperatureC) {
        saturation[port] = is;
        invNVt[port] = 1.0 / (emission * thermalVoltage(temperatureC));
    }
};

// Is from 25 C to temperatureC: T^3 exp(-Eg / (n Vt)).
inline double saturationScale(double temperatureC, double emission, double bandgap) {
    double vt0 = thermalVoltage(25.0), vt = thermalVoltage(temperatureC);
    return std::pow((temperatureC + 273.15) / 298.15, 3.0)
         * std::exp(bandgap / (emission * vt0) - bandgap / (emission * vt));
}

// Germanium leakage doubles every this many degrees. Hand-tunable.
static const double kLeakDoublingC = 9.0;
inline double leakageAt(double icbo25, double temperatureC) {
    return icbo25 * std::pow(2.0, (temperatureC - 25.0) / kLeakDoublingC);
}

// ==========================================================================
// DK matrices
//
// Every reactive element k is a companion: a conductance g_k with a history
// source x_k. Its element current (a -> b) is
//     i_k[n] = g_k v_k[n] + historySign_k * x_k[n-1]
//     x_k[n] = voltageGain * g_k v_k[n] + historyGain_k * x_k[n-1]
// Trapezoidal: g = 2C/T (sign -1) or T/(2L) (sign +1), voltageGain 2,
// historyGain = sign. Backward Euler, only for the DC operating point
// because it is L-stable at a huge T: g = C/T or T/L, voltageGain 1,
// historyGain 0 for a capacitor and +1 for an inductor.
//
// Node voltages are then linear in (x, u, i):
//     x[n] = A x[n-1] + B u + C i
//     v    = D x[n-1] + E u + F i      (junction voltages, Newton)
//     z    = Dz x[n-1] + Ez u + Fz i   (the stage's output node)
// ==========================================================================
template <int N, int S, int P, int I>
struct DKMatrices {
    double A[S][S], B[S][I], C[S][P];
    double D[P][S], E[P][I], F[P][P];
    double Dz[S], Ez[I], Fz[P];
    double stateG[S], historyGain[S];
    bool   stateIsCap[S];
};

// The nodal matrix G holds conductances only (resistors, companions, gmin),
// so it is symmetric positive definite: Cholesky, G = L L^T, no pivoting.
// Each row the step needs (a companion's terminals, a junction's terminals,
// the output node) is solved against G once, then projected onto the sparse
// source columns, rather than inverting G and multiplying out in full.
template <int N, int S, int P, int I>
inline bool buildDK(const Netlist& net, int outputNode, double T, DKMatrices<N, S, P, I>& m,
                    bool backwardEuler = false) {
    double G[N][N] = {};
    const double voltageGain = backwardEuler ? 1.0 : 2.0;
    // Sparse source columns: each companion's history current between its
    // two nodes, each current source into its node, each junction port's
    // injection into its terminals; and the rows that read each port.
    struct Tap { int node, column; double gain; };
    int    stateA[S], stateB[S];
    double stateSign[S];
    Tap sources[MAX_ELEMENTS];
    Tap injections[6 * MAX_ELEMENTS];
    Tap portRows[4 * MAX_ELEMENTS];
    int sourceCount = 0, injectionCount = 0, portRowCount = 0;
    auto inject = [&](int node, int port, double gain) { if (node != GND) injections[injectionCount++] = { node, port, gain }; };
    auto readPort = [&](int node, int port, double gain) { if (node != GND) portRows[portRowCount++] = { node, port, gain }; };

    auto stampG = [&](int a, int b, double g) {
        if (a != GND) G[a][a] += g;
        if (b != GND) G[b][b] += g;
        if (a != GND && b != GND) { G[a][b] -= g; G[b][a] -= g; }
    };

    std::memset(&m, 0, sizeof(m));
    int states = 0;
    for (int idx = 0; idx < net.count; ++idx) {
        const Element& e = net.el[idx];
        switch (e.type) {
        case EL_RES: stampG(e.a, e.b, 1.0 / e.value); break;
        case EL_CAP:
        case EL_IND: {
            if (states >= S) return false;
            bool isCap = (e.type == EL_CAP);
            double g;
            if (backwardEuler) g = isCap ? e.value / T : T / e.value;
            else               g = isCap ? 2.0 * e.value / T : T / (2.0 * e.value);
            double sign = isCap ? -1.0 : 1.0;
            stampG(e.a, e.b, g);
            // The history term is a current sign * x flowing a -> b; it leaves
            // node a, so it enters the right-hand side as -sign at a.
            stateA[states] = e.a;
            stateB[states] = e.b;
            stateSign[states] = sign;
            m.stateG[states] = g;
            m.historyGain[states] = (backwardEuler && isCap) ? 0.0 : sign;
            m.stateIsCap[states] = isCap;
            ++states;
            break;
        }
        case EL_ISRC:
            if (e.a != GND && e.input >= 0 && e.input < I) sources[sourceCount++] = { e.a, e.input, e.value };
            break;
        case EL_BJT: {
            // PNP: vF = V(E) - V(B), vR = V(C) - V(B); the forward diode current
            // leaves the emitter, alphaF of it arrives at the collector. NPN is
            // the same with every sign flipped.
            const double pol = e.polarity;
            const int portF = e.port, portR = e.port + 1;
            if (portR >= P) return false;
            readPort(e.emitter, portF, pol);
            readPort(e.collector, portR, pol);
            readPort(e.base, portF, -pol);
            readPort(e.base, portR, -pol);
            inject(e.emitter, portF, -pol);
            inject(e.emitter, portR, pol * e.alphaR);
            inject(e.collector, portF, pol * e.alphaF);
            inject(e.collector, portR, -pol);
            inject(e.base, portF, pol * (1.0 - e.alphaF));
            inject(e.base, portR, pol * (1.0 - e.alphaR));
            break;
        }
        case EL_DIODE:
            if (e.port >= P) return false;
            readPort(e.a, e.port, 1.0);
            readPort(e.b, e.port, -1.0);
            inject(e.a, e.port, -1.0);
            inject(e.b, e.port, 1.0);
            break;
        }
    }
    if (states != S) return false;
    for (int node = 0; node < N; ++node) G[node][node] += 1e-12;   // gmin

    // Cholesky, lower triangle, with the reciprocal of its diagonal.
    double lower[N][N], invDiagonal[N];
    for (int j = 0; j < N; ++j) {
        double d = G[j][j];
        for (int k = 0; k < j; ++k) d -= lower[j][k] * lower[j][k];
        if (!(d > 1e-300)) return false;
        double pivot = std::sqrt(d);
        lower[j][j] = pivot;
        invDiagonal[j] = 1.0 / pivot;
        for (int i = j + 1; i < N; ++i) {
            double sum = G[i][j];
            for (int k = 0; k < j; ++k) sum -= lower[i][k] * lower[j][k];
            lower[i][j] = sum * invDiagonal[j];
        }
    }

    // Every row the step reads, solved against G together: as columns of
    // one right-hand side, so each elimination runs across all of them at
    // once (vectorizes). Columns: the S companions, the P ports, the output.
    static const int SOLVED = S + P + 1;
    static const int SOLVED_PAD = (SOLVED + 1) / 2 * 2;
    alignas(16) double solved[N][SOLVED_PAD];
    std::memset(solved, 0, sizeof(solved));
    for (int s = 0; s < S; ++s) {
        const double scale = voltageGain * m.stateG[s];
        if (stateA[s] != GND) solved[stateA[s]][s] += scale;
        if (stateB[s] != GND) solved[stateB[s]][s] -= scale;
    }
    for (int k = 0; k < portRowCount; ++k) solved[portRows[k].node][S + portRows[k].column] += portRows[k].gain;
    if (outputNode != GND) solved[outputNode][S + P] = 1.0;
    // G is symmetric, so row G^-1 is the solve G w = row^T: forward through
    // L, then back through L^T.
    for (int i = 0; i < N; ++i) {
        for (int k = 0; k < i; ++k) {
            const double factor = lower[i][k];
            if (factor == 0.0) continue;
            for (int c = 0; c < SOLVED_PAD; ++c) solved[i][c] -= factor * solved[k][c];
        }
        for (int c = 0; c < SOLVED_PAD; ++c) solved[i][c] *= invDiagonal[i];
    }
    for (int i = N - 1; i >= 0; --i) {
        for (int k = i + 1; k < N; ++k) {
            const double factor = lower[k][i];
            if (factor == 0.0) continue;
            for (int c = 0; c < SOLVED_PAD; ++c) solved[i][c] -= factor * solved[k][c];
        }
        for (int c = 0; c < SOLVED_PAD; ++c) solved[i][c] *= invDiagonal[i];
    }

    // Each solved row onto the sparse source columns.
    auto project = [&](int column, double* outX, double* outU, double* outI) {
        for (int s = 0; s < S; ++s) {
            double sum = 0.0;
            if (stateA[s] != GND) sum -= stateSign[s] * solved[stateA[s]][column];
            if (stateB[s] != GND) sum += stateSign[s] * solved[stateB[s]][column];
            outX[s] = sum;
        }
        for (int k = 0; k < sourceCount; ++k) outU[sources[k].column] += sources[k].gain * solved[sources[k].node][column];
        for (int k = 0; k < injectionCount; ++k) outI[injections[k].column] += injections[k].gain * solved[injections[k].node][column];
    };
    for (int s = 0; s < S; ++s) {
        project(s, m.A[s], m.B[s], m.C[s]);
        m.A[s][s] += m.historyGain[s];
    }
    for (int p = 0; p < P; ++p) project(S + p, m.D[p], m.E[p], m.F[p]);
    project(S + P, m.Dz, m.Ez, m.Fz);
    return true;
}

// ==========================================================================
// Core: one DK step per oversampled sample.
//
//   - exps in float_4 (one call per four junctions per iteration); residual,
//     Jacobian, PxP solve, limiter and state in double
//   - tangent predictor: dv = -J^-1 dp from the last factorization
//   - SPICE pnjlim on each junction (rises past vcrit compressed)
//   - stop when no junction current moved more than kToleranceAmps
// ==========================================================================

// Newton stop, amps of junction current change. Hand-tunable. Against a
// 1 nA reference: 10 nA -146 dB at 1.73 iterations, 1 uA -101 dB at 1.13.
static const double kToleranceAmps = 1e-6;
// Iteration cap. Most samples take 1-2; capping lower corrupts the state.
static const int kMaxIterations = 16;

template <int N, int S, int P, int I>
struct Core {
    static const int PADDED = (P + 3) / 4 * 4;    // ports rounded up to whole float_4s
    // Rows of the state update: the S states, then the output; padded to an
    // even count (and the ports likewise) so every column is whole pairs.
    static const int ROWS = S + 1;
    static const int ROWS_PAD = (ROWS + 1) / 2 * 2;
    static const int PORTS_PAD = (P + 1) / 2 * 2;

    // Everything a substep reads, by column, so each product runs as
    // contiguous multiply-adds down a column (vectorizes). One flat block of
    // doubles, so a pot move can glide it in with a single loop.
    struct Packed {
        alignas(16) double update[S + P][ROWS_PAD];   // [A C; Dz Fz]
        alignas(16) double port[S][PORTS_PAD];        // D
        alignas(16) double inputUpdate[I][ROWS_PAD];  // [B; Ez]
        alignas(16) double inputPort[I][PORTS_PAD];   // E
        alignas(16) double F[P][PORTS_PAD];
    };
    static const int kPackedCount = (S + P) * ROWS_PAD + S * PORTS_PAD + I * ROWS_PAD + I * PORTS_PAD + P * PORTS_PAD;
    static_assert(sizeof(Packed) == kPackedCount * sizeof(double), "Packed must be one gapless block of doubles");
    static double* flat(Packed& block) { return &block.update[0][0]; }

    DKMatrices<N, S, P, I> m;          // as built; the substeps run on `packed`
    Packed packed;
    double x[S] = {};
    double v[P] = {};
    double current[P] = {};
    double saturation[P] = {}, invNVt[P] = {}, nVt[P] = {}, vcrit[P] = {};
    float  invNVtFloat[PADDED] = {};
    // Last factorization, for the predictor: U above the diagonal, the
    // reciprocal of its diagonal, and the elimination multipliers.
    double factorU[P][P] = {}, invDiagonal[P] = {}, multiplier[P][P] = {};
    double pPrev[P] = {};
    bool   haveFactor = false;

    // Rail and leakage enter once per control block (setHeldInputs), the
    // noise once per sample (prepare); only the signal moves per substep.
    // A held change glides in over kHeldGlideSamples base samples (one
    // control block): the drain ramp moves the rail, and a stepped rail
    // through these gains clicks.
    static const int kHeldGlideSamples = 32;
    alignas(16) double heldUpdate[ROWS_PAD] = {}, heldPort[PORTS_PAD] = {};
    alignas(16) double sampleUpdate[ROWS_PAD] = {}, samplePort[PORTS_PAD] = {};
    double heldU[I] = {}, heldTarget[I] = {}, glideU[I] = {};
    int    glideLeft = 0;
    // A pot move rebuilds the matrices; the new ones glide in over the same
    // block, entry by entry, so a turning pot moves the circuit smoothly
    // instead of in control-rate steps (zipper noise).
    Packed packedTarget, packedStep;
    int    matrixGlideLeft = 0;

    double z = 0.0;
    int    lastIterations = 0;

    // Junction constants move with temperature (drain). Like the held
    // inputs they glide in over a control block; snap = true jumps.
    double glideSaturation[P] = {}, glideInvNVt[P] = {};
    int    portGlideLeft = 0;

    Core() { std::memset(&packed, 0, sizeof(packed)); }

    void updatePortDerived() {
        for (int k = 0; k < P; ++k) {
            nVt[k] = 1.0 / invNVt[k];
            vcrit[k] = nVt[k] * std::log(nVt[k] / (1.41421356237 * saturation[k]));
            invNVtFloat[k] = (float)invNVt[k];
        }
    }

    void setPorts(const PortModel<P>& pm, bool snap = false) {
        bool moved = false;
        for (int k = 0; k < P; ++k)
            moved = moved || pm.saturation[k] != saturation[k] || pm.invNVt[k] != invNVt[k];
        if (!moved) return;
        if (snap) {
            for (int k = 0; k < P; ++k) { saturation[k] = pm.saturation[k]; invNVt[k] = pm.invNVt[k]; }
            portGlideLeft = 0;
            updatePortDerived();
            return;
        }
        for (int k = 0; k < P; ++k) {
            glideSaturation[k] = (pm.saturation[k] - saturation[k]) / kHeldGlideSamples;
            glideInvNVt[k] = (pm.invNVt[k] - invNVt[k]) / kHeldGlideSamples;
        }
        portGlideLeft = kHeldGlideSamples;
    }

    static void pack(const DKMatrices<N, S, P, I>& from, Packed& to) {
        std::memset(&to, 0, sizeof(to));
        for (int c = 0; c < S; ++c) {
            for (int r = 0; r < S; ++r) to.update[c][r] = from.A[r][c];
            to.update[c][S] = from.Dz[c];
            for (int r = 0; r < P; ++r) to.port[c][r] = from.D[r][c];
        }
        for (int c = 0; c < P; ++c) {
            for (int r = 0; r < S; ++r) to.update[S + c][r] = from.C[r][c];
            to.update[S + c][S] = from.Fz[c];
        }
        for (int c = 0; c < I; ++c) {
            for (int r = 0; r < S; ++r) to.inputUpdate[c][r] = from.B[r][c];
            to.inputUpdate[c][S] = from.Ez[c];
            for (int r = 0; r < P; ++r) to.inputPort[c][r] = from.E[r][c];
        }
        for (int r = 0; r < P; ++r)
            for (int c = 0; c < P; ++c) to.F[r][c] = from.F[r][c];
    }

    // The held inputs' share of the update and the ports at the current matrices.
    void updateHeld() {
        for (int r = 0; r < ROWS_PAD; ++r) {
            double a = 0.0;
            for (int c = INPUT_RAIL; c < I - 1; ++c) a += packed.inputUpdate[c][r] * heldU[c];
            heldUpdate[r] = a;
        }
        for (int r = 0; r < PORTS_PAD; ++r) {
            double a = 0.0;
            for (int c = INPUT_RAIL; c < I - 1; ++c) a += packed.inputPort[c][r] * heldU[c];
            heldPort[r] = a;
        }
    }

    // snap = true jumps straight to the new values and repacks `m`, which
    // the caller may have replaced outright (start, DC solve). Otherwise only
    // a change glides in; the same inputs again cost nothing.
    void setHeldInputs(const double u[I], bool snap = false) {
        if (snap) {
            pack(m, packed);
            matrixGlideLeft = 0;
            for (int c = 0; c < I; ++c) { heldU[c] = heldTarget[c] = u[c]; glideU[c] = 0.0; }
            glideLeft = 0;
            updateHeld();
            return;
        }
        bool changed = false;
        for (int c = INPUT_RAIL; c < I - 1; ++c) changed = changed || u[c] != heldTarget[c];
        if (!changed) return;
        for (int c = INPUT_RAIL; c < I - 1; ++c) {
            heldTarget[c] = u[c];
            glideU[c] = (u[c] - heldU[c]) / kHeldGlideSamples;
        }
        glideLeft = kHeldGlideSamples;
    }

    // New matrices after a pot move, glided in over a control block.
    void glideToMatrices(const DKMatrices<N, S, P, I>& next) {
        m = next;
        pack(next, packedTarget);
        const double perSample = 1.0 / kHeldGlideSamples;
        double* step = flat(packedStep);
        const double* from = flat(packed);
        const double* to = flat(packedTarget);
        for (int k = 0; k < kPackedCount; ++k) step[k] = (to[k] - from[k]) * perSample;
        matrixGlideLeft = kHeldGlideSamples;
    }

    void prepare(double noise) {
        if (portGlideLeft > 0) {
            --portGlideLeft;
            for (int k = 0; k < P; ++k) { saturation[k] += glideSaturation[k]; invNVt[k] += glideInvNVt[k]; }
            updatePortDerived();
        }
        bool moved = false;
        if (matrixGlideLeft > 0) {
            if (--matrixGlideLeft == 0) packed = packedTarget;
            else {
                double* entry = flat(packed);
                const double* step = flat(packedStep);
                for (int k = 0; k < kPackedCount; ++k) entry[k] += step[k];
            }
            moved = true;
        }
        if (glideLeft > 0) {
            if (--glideLeft == 0) for (int c = INPUT_RAIL; c < I - 1; ++c) heldU[c] = heldTarget[c];
            else                  for (int c = INPUT_RAIL; c < I - 1; ++c) heldU[c] += glideU[c];
            moved = true;
        }
        if (moved) updateHeld();
        for (int r = 0; r < PORTS_PAD; ++r) samplePort[r] = heldPort[r] + packed.inputPort[I - 1][r] * noise;
        for (int r = 0; r < ROWS_PAD; ++r) sampleUpdate[r] = heldUpdate[r] + packed.inputUpdate[I - 1][r] * noise;
    }

    inline double step(double signal) {
        double p[P];
        for (int r = 0; r < P; ++r) p[r] = samplePort[r] + packed.inputPort[INPUT_SIGNAL][r] * signal;
        for (int c = 0; c < S; ++c) {
            const double state = x[c];
            for (int r = 0; r < P; ++r) p[r] += packed.port[c][r] * state;
        }

        // Tangent predictor.
        if (haveFactor) {
            double y[P];
            for (int r = 0; r < P; ++r) y[r] = p[r] - pPrev[r];
            for (int col = 0; col < P; ++col)
                for (int r = col + 1; r < P; ++r) y[r] -= multiplier[r][col] * y[col];
            double stepV[P];
            for (int r = P - 1; r >= 0; --r) {
                double s = y[r];
                for (int c = r + 1; c < P; ++c) s -= factorU[r][c] * stepV[c];
                stepV[r] = s * invDiagonal[r];
            }
            for (int k = 0; k < P; ++k) {
                double vNew = v[k] - stepV[k];
                if (vNew > vcrit[k] && vNew - v[k] > 2.0 * nVt[k])
                    vNew = (v[k] > 0.0) ? v[k] + nVt[k] * std::log(1.0 + (vNew - v[k]) / nVt[k])
                                        : nVt[k] * std::log(vNew / nVt[k]);
                if (std::isfinite(vNew)) v[k] = vNew;
            }
        }
        for (int r = 0; r < P; ++r) pPrev[r] = p[r];

        double slope[P], dv[P] = {};
        for (int it = 0; it < kMaxIterations; ++it) {
            for (int k0 = 0; k0 < P; k0 += 4) {
                float lanes[4] = { 0.f, 0.f, 0.f, 0.f };
                for (int k = k0; k < k0 + 4 && k < P; ++k) lanes[k - k0] = (float)v[k];
                float_4 arg = rack::simd::clamp(float_4::load(lanes) * float_4::load(&invNVtFloat[k0]),
                                                float_4(-40.f), float_4(40.f));
                float_4 e = rack::simd::exp(arg);
                for (int k = k0; k < k0 + 4 && k < P; ++k) {
                    current[k] = saturation[k] * ((double)e[k - k0] - 1.0);
                    slope[k]   = saturation[k] * invNVt[k] * (double)e[k - k0];
                }
            }
            // J = F diag(slope) - I, factored in place into factorU.
            double (&J)[P][P] = factorU;
            double rhs[P];
            for (int r = 0; r < P; ++r) {
                double g = p[r] - v[r];
                for (int c = 0; c < P; ++c) { g += packed.F[r][c] * current[c]; J[r][c] = packed.F[r][c] * slope[c]; }
                J[r][r] -= 1.0;
                rhs[r] = -g;
            }
            // Fixed-order elimination; pivoting made no difference anywhere.
            for (int col = 0; col < P; ++col) {
                double inv = 1.0 / J[col][col];
                invDiagonal[col] = inv;
                for (int r = col + 1; r < P; ++r) {
                    double f = J[r][col] * inv;
                    multiplier[r][col] = f;
                    for (int c = col + 1; c < P; ++c) J[r][c] -= f * J[col][c];
                    rhs[r] -= f * rhs[col];
                }
            }
            haveFactor = true;
            for (int r = P - 1; r >= 0; --r) {
                double s = rhs[r];
                for (int c = r + 1; c < P; ++c) s -= J[r][c] * dv[c];
                dv[r] = s * invDiagonal[r];
            }
            double maxMove = 0.0;
            for (int k = 0; k < P; ++k) {
                double vNew = v[k] + dv[k];
                // SPICE pnjlim: a rise past vcrit is compressed, falls are free.
                if (vNew > vcrit[k] && std::fabs(dv[k]) > 2.0 * nVt[k]) {
                    if (v[k] > 0.0) {
                        double a = 1.0 + dv[k] / nVt[k];
                        vNew = (a > 0.0) ? v[k] + nVt[k] * std::log(a) : vcrit[k];
                    } else {
                        vNew = nVt[k] * std::log(vNew / nVt[k]);
                    }
                }
                dv[k] = vNew - v[k];
                maxMove = std::fmax(maxMove, std::fabs(slope[k] * dv[k]));
                v[k] = vNew;
            }
            lastIterations = it + 1;
            if (maxMove < kToleranceAmps) break;
        }
        // Currents at the final voltages, linearized from the last evaluation.
        for (int k = 0; k < P; ++k) current[k] += slope[k] * dv[k];

        // New states and the output in one pass over the columns.
        alignas(16) double next[ROWS_PAD];
        for (int r = 0; r < ROWS_PAD; ++r) next[r] = sampleUpdate[r] + packed.inputUpdate[INPUT_SIGNAL][r] * signal;
        for (int c = 0; c < S; ++c) {
            const double state = x[c];
            for (int r = 0; r < ROWS_PAD; ++r) next[r] += packed.update[c][r] * state;
        }
        for (int c = 0; c < P; ++c) {
            const double junction = current[c];
            for (int r = 0; r < ROWS_PAD; ++r) next[r] += packed.update[S + c][r] * junction;
        }
        z = next[S];
        for (int r = 0; r < S; ++r) x[r] = next[r];
        return z;
    }

    // A full step with every input held at u (DC solve).
    void stepHeld(const double u[I]) {
        setHeldInputs(u, true);
        prepare(u[I - 1]);
        step(u[INPUT_SIGNAL]);
    }

    bool finite() const {
        if (!std::isfinite(z)) return false;
        for (int k = 0; k < P; ++k) if (!std::isfinite(v[k])) return false;
        for (int s = 0; s < S; ++s) if (!std::isfinite(x[s])) return false;
        return true;
    }
};

// ==========================================================================
// DC operating point
//
// The same Core, run on backward-Euler matrices at a 10 s step: capacitors
// open, inductors shorted. From cold the supply is ramped up over 50 steps;
// warm (the previous point as the start) it settles in a few. The state is
// then written for the trapezoidal audio matrices:
//   capacitor: v = x_BE / g_BE, zero current; x_trap = g_trap v
//   inductor : v = 0, current = x_BE;          x_trap = current
// a steady state for both kinds.
// ==========================================================================
template <int N, int S, int P, int I>
inline bool solveOperatingPoint(const Netlist& net, int outputNode, const PortModel<P>& pm,
                                const double u[I], Core<N, S, P, I>& scratch, bool warm) {
    DKMatrices<N, S, P, I> dcm;
    if (!buildDK<N, S, P, I>(net, outputNode, 10.0, dcm, true)) return false;
    if (!warm) {
        std::memset(scratch.x, 0, sizeof(scratch.x));
        std::memset(scratch.v, 0, sizeof(scratch.v));
    }
    scratch.m = dcm;
    scratch.setPorts(pm, true);
    const int ramp = warm ? 0 : 50;
    const int settle = warm ? 6 : 30;
    for (int s = 0; s < ramp + settle; ++s) {
        double scale = (s < ramp) ? (double)(s + 1) / ramp : 1.0;
        double uStep[I];
        for (int c = 0; c < I; ++c) uStep[c] = u[c] * scale;
        scratch.haveFactor = false;
        scratch.stepHeld(uStep);
    }
    return scratch.finite();
}

// Moves the state solved on backward-Euler matrices (in `dc`) onto the
// trapezoidal audio matrices already in `core.m`.
template <int N, int S, int P, int I>
inline void loadOperatingPoint(Core<N, S, P, I>& core, const Core<N, S, P, I>& dc) {
    for (int s = 0; s < S; ++s) {
        double xs = dc.m.stateIsCap[s] ? core.m.stateG[s] * dc.x[s] / dc.m.stateG[s] : dc.x[s];
        core.x[s] = xs;
    }
    for (int k = 0; k < P; ++k) { core.v[k] = dc.v[k]; core.current[k] = dc.current[k]; }
    core.z = dc.z;
    core.haveFactor = false;
}

// ==========================================================================
// Fixed 4x polyphase resampler, mono.
// Kaiser-windowed sinc, 16 base samples long, cutoff 0.45 of the base rate.
// ==========================================================================
static const int kOversample = 4;
static const int kResampleQuality = 16;                     // base samples per kernel
static const int kResampleTaps = kOversample * kResampleQuality;

struct Resampler {
    // Up: per phase, its 16 taps as four float_4, newest first, against a
    // doubled ring of base-rate samples (index walks down).
    float_4 upPhase[kOversample][kResampleQuality / 4];
    float   upRing[2 * kResampleQuality] = {};
    int     upIndex = 0;
    // Down: the kernel as float_4, newest first, against a doubled ring of
    // oversampled samples (index walks down).
    float_4 downKernel[kResampleTaps / 4];
    float   downRing[2 * kResampleTaps] = {};
    int     downIndex = 0;

    Resampler() {
        float kernel[kResampleTaps];
        const double beta = 8.0, cutoff = 0.45;
        auto bessel0 = [](double x) {
            double sum = 1.0, term = 1.0;
            for (int k = 1; k < 30; ++k) { term *= (x / (2.0 * k)) * (x / (2.0 * k)); sum += term; }
            return sum;
        };
        double fc = cutoff / kOversample, total = 0.0;
        for (int i = 0; i < kResampleTaps; ++i) {
            double m = i - (kResampleTaps - 1) / 2.0;
            double sinc = (m == 0.0) ? 2.0 * fc : std::sin(2.0 * M_PI * fc * m) / (M_PI * m);
            double r = 2.0 * i / (kResampleTaps - 1) - 1.0;
            double w = bessel0(beta * std::sqrt(std::fmax(0.0, 1.0 - r * r))) / bessel0(beta);
            kernel[i] = (float)(sinc * w);
            total += kernel[i];
        }
        for (int i = 0; i < kResampleTaps; ++i) kernel[i] = (float)(kernel[i] / total);
        for (int phase = 0; phase < kOversample; ++phase)
            for (int j = 0; j < kResampleQuality; j += 4)
                upPhase[phase][j / 4] = float_4(kernel[j * kOversample + phase], kernel[(j + 1) * kOversample + phase],
                                                kernel[(j + 2) * kOversample + phase], kernel[(j + 3) * kOversample + phase])
                                      * (float)kOversample;
        for (int i = 0; i < kResampleTaps; i += 4)
            downKernel[i / 4] = float_4(kernel[i], kernel[i + 1], kernel[i + 2], kernel[i + 3]);
    }

    void reset() {
        std::memset(upRing, 0, sizeof(upRing));
        std::memset(downRing, 0, sizeof(downRing));
        upIndex = downIndex = 0;
    }

    // One base sample in, kOversample out.
    void up(float in, float out[kOversample]) {
        upIndex = (upIndex + kResampleQuality - 1) % kResampleQuality;
        upRing[upIndex] = in;
        upRing[upIndex + kResampleQuality] = in;
        const float* window = &upRing[upIndex];
        float_4 w0 = float_4::load(window), w1 = float_4::load(window + 4),
                w2 = float_4::load(window + 8), w3 = float_4::load(window + 12);
        for (int phase = 0; phase < kOversample; ++phase) {
            float_4 acc = w0 * upPhase[phase][0] + w1 * upPhase[phase][1] + w2 * upPhase[phase][2] + w3 * upPhase[phase][3];
            out[phase] = acc[0] + acc[1] + acc[2] + acc[3];
        }
    }

    // kOversample in (oldest first), one base sample out.
    float down(const float in[kOversample]) {
        for (int s = 0; s < kOversample; ++s) {
            downIndex = (downIndex + kResampleTaps - 1) % kResampleTaps;
            downRing[downIndex] = in[s];
            downRing[downIndex + kResampleTaps] = in[s];
        }
        const float* window = &downRing[downIndex];
        float_4 acc = float_4(0.f);
        for (int i = 0; i < kResampleTaps / 4; ++i) acc += float_4::load(window + 4 * i) * downKernel[i];
        return acc[0] + acc[1] + acc[2] + acc[3];
    }
};

// ==========================================================================
// Noise
// ==========================================================================
static const double kBoltzmann = 1.380649e-23;
static const double kElectronCharge = 1.602176634e-19;

struct NoiseSource {
    uint32_t state = 0x1234567u;
    // Pink filter state (Paul Kellet's three-pole approximation).
    float pinkA = 0.f, pinkB = 0.f, pinkC = 0.f;

    void seed(uint32_t s) { state = s * 2654435761u + 0x9E3779B9u; if (!state) state = 1; }

    // Unit-variance, near-Gaussian: the sum of four uniforms (Irwin-Hall),
    // centred and scaled, the four taken as 16-bit halves of two draws.
    // Tails stop at +-3.5 sigma, which a noise floor never shows.
    inline float gaussian() {
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        const uint32_t first = state;
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        const uint32_t second = state;
        float sum = (float)(first & 0xFFFFu) + (float)(first >> 16) + (float)(second & 0xFFFFu) + (float)(second >> 16);
        return ((sum + 2.f) * (1.f / 65536.f) - 2.f) * 1.7320508f;
    }
    // Pink noise from unit white; density at 1 kHz is kPinkGainAt1kHz times
    // the white density at 48 kHz.
    inline float pink(float white) {
        pinkA = 0.99765f * pinkA + white * 0.0990460f;
        pinkB = 0.96300f * pinkB + white * 0.2965164f;
        pinkC = 0.57000f * pinkC + white * 1.0526913f;
        return pinkA + pinkB + pinkC + white * 0.1848f;
    }
};
static const float kPinkGainAt1kHz = 4.66f;

// ==========================================================================
// MODELS
//
// Every model is its schematic as drawn, with the same passive-pickup front
// end (single coil, guitar volume pot, cable) in front of its input
// capacitor. Germanium circuits are PNP with a negative rail; the
// Super-Fuzz is NPN silicon with a positive rail.
// ==========================================================================

// Settings every model reads at control rate.
struct ModelControls {
    double fuzz = 1.0;           // 0..1, the model's own gain control
    double guitar = 1.0;         // guitar pot wiper, 0..1 (audio taper already applied)
    double drain = 0.0;          // 0 fresh battery .. 1 dead
};

// One transistor or diode type.
struct Device {
    bool   germanium;
    double saturation;           // Is at 25 C, amps
    double emission;             // n
    double betaF, betaR;
    double leakage;              // Icbo at 25 C, amps (germanium only)
};
// Hand-tunable device sets. Germanium leakage is per model (it sets the
// bias of the circuits that have no trimmer).
static const Device kGermanium = { true,  1e-6,  1.1, 90.0,  5.0, 2e-6 };
static const Device kSilicon   = { false, 2e-14, 1.0, 200.0, 5.0, 0.0 };
static const double kGermaniumBandgap = 0.66, kSiliconBandgap = 1.12;
// OA90 point-contact germanium diode.
static const double kDiodeSaturation = 3e-7, kDiodeEmission = 1.5;
// Collector-base junction capacitance (always present, keeps edges finite).
static const double kGermaniumCjc = 15e-12, kSiliconCjc = 4e-12;
// Base spreading resistance of the germanium transistors, ohms.
static const double kBaseResistance = 150.0;

// Passive single-coil pickup through the guitar's volume pot and a cable.
static const double kPickupOhms = 6000.0, kPickupHenries = 2.5;
static const double kGuitarPotOhms = 250e3, kCableFarads = 500e-12;
// Battery internal resistance, ohms (the rail is a Norton source).
static const double kBatteryOhms = 1.0;

inline void addPickup(Netlist& net, int coilTop, int coilBottom, int wiper, double guitar) {
    net.isrc(coilTop, 1.0 / kPickupOhms, INPUT_SIGNAL);
    net.res(coilTop, GND, kPickupOhms);
    net.ind(coilTop, coilBottom, kPickupHenries);
    double fraction = std::fmin(std::fmax(guitar, 0.0), 1.0);
    net.res(coilBottom, wiper, (1.0 - fraction) * kGuitarPotOhms);
    net.res(wiper, GND, fraction * kGuitarPotOhms);
    net.cap(wiper, GND, kCableFarads);
}
inline void addRail(Netlist& net, int rail) {
    net.isrc(rail, 1.0 / kBatteryOhms, INPUT_RAIL);
    net.res(rail, GND, kBatteryOhms);
}
// A germanium PNP with its spreading resistance, Cjc and leakage source.
// Leakage flows base -> collector inside the device, so it is drawn from
// the internal base and pushed out of the collector; the circuit amplifies
// it to (beta + 1) Icbo on its own.
inline void addGermaniumPnp(Netlist& net, int emitter, int base, int baseInternal, int collector,
                            const Device& d, int port, int leakInput, double extraBaseCollectorFarads = 0.0) {
    net.res(base, baseInternal, kBaseResistance);
    net.cap(base, collector, kGermaniumCjc + extraBaseCollectorFarads);
    net.bjt(emitter, baseInternal, collector, true, d.betaF, d.betaR, port);
    net.isrc(collector, 1.0, leakInput);
    net.isrc(baseInternal, -1.0, leakInput);
}
// Drives `node` with the voltage in u[INPUT_SIGNAL] through `ohms` (a
// Norton source), for a stage fed by the stage before it.
inline void addDrive(Netlist& net, int node, double ohms) {
    net.isrc(node, 1.0 / ohms, INPUT_SIGNAL);
    net.res(node, GND, ohms);
}

// Interpolates a 0..1 knob through an 11-point table.
inline double lookup11(const double table[11], double knob) {
    double scaled = std::fmin(std::fmax(knob, 0.0), 1.0) * 10.0;
    int i = std::min(9, (int)scaled);
    return table[i] + (scaled - i) * (table[i + 1] - table[i]);
}

// --------------------------------------------------------------------------
// A circuit stage: netlist, matrices, Newton core, DC solver.
// --------------------------------------------------------------------------
template <int N, int S, int P, int I>
struct Stage {
    Netlist net;
    int output = GND;
    PortModel<P> ports;
    Core<N, S, P, I> core;
    Core<N, S, P, I> scratch;          // DC solve
    double u[I] = {};                  // held inputs (rail, leakage)
    double rest = 0.0;                 // output at the operating point

    bool start(double T, double signalAtRest, bool warm) {
        u[INPUT_SIGNAL] = signalAtRest;
        u[I - 1] = 0.0;
        if (!buildDK<N, S, P, I>(net, output, T, core.m)) return false;
        if (!solveOperatingPoint<N, S, P, I>(net, output, ports, u, scratch, warm)) return false;
        loadOperatingPoint(core, scratch);
        core.setPorts(ports, true);
        core.setHeldInputs(u, true);
        rest = scratch.z;
        return true;
    }
    // New matrices after a pot moved. Only resistors move, so every
    // companion keeps its g and the state carries over unchanged.
    bool rebuild(double T) {
        DKMatrices<N, S, P, I> next;
        if (!buildDK<N, S, P, I>(net, output, T, next)) return false;
        core.glideToMatrices(next);
        core.setHeldInputs(u);
        return true;
    }
    void refresh() {
        core.setPorts(ports);
        core.setHeldInputs(u);
    }
};

// The display's view of a simulated device.
enum DeviceKind { DEVICE_PNP, DEVICE_NPN, DEVICE_NJFET, DEVICE_DIODE };
static const int kMaxDevices = 8;

// Base class the module drives. One instance per model; only the selected
// one (two while crossfading) runs.
struct FuzzModel {
    virtual ~FuzzModel() {}
    // Control rate: rewrite the netlists and inputs from the controls.
    // Returns true if a pot moved enough that the matrices need rebuilding.
    virtual bool configure(const ModelControls& c) = 0;
    virtual bool start(double T, bool warm) = 0;   // matrices + DC operating point
    virtual void rebuild(double T) = 0;
    virtual void refresh() = 0;                      // ports and held inputs only
    virtual void prepare(double noise) = 0;
    // Base rate, before the upsampler: a front stage that stays clean enough
    // to run without oversampling. Returns what the substeps are fed.
    virtual double front(double emf) { return emf; }
    virtual double step(double emf) = 0;             // one substep; output minus its rest value
    virtual bool finite() const = 0;
    virtual double noiseScale() const = 0;           // amps of base noise current per unit noise
    // Simulated devices, for the display: how many, what each is, and
    // their junction currents now (amps). `forward` is the main junction
    // (base-emitter, the JFET channel, a diode), `reverse` the base-collector
    // junction (positive when the transistor saturates; 0 if none).
    virtual int deviceCount() const = 0;
    virtual DeviceKind deviceKind(int index) const = 0;
    virtual void sense(float forward[kMaxDevices], float reverse[kMaxDevices]) const = 0;
};

// Drain: the battery sags and the junctions warm together, the warming in
// step with the lost voltage (heatC more at a fully flat battery).
inline double drainTemperature(double railFraction, double heatC) { return 25.0 + heatC * (1.0 - railFraction); }

// Battery voltage as a fraction of fresh, for BATTERY's drain 0..1. Two
// straight segments: a plain sag down to `onset` (where the circuit is 3 dB
// down) over the first kDrainOnsetAt of the travel, then the sputter and
// death down to `floor` over the rest, so every model spends most of the knob
// in the interesting part however early or late it gives out.
static const double kDrainOnsetAt = 0.3;
inline double railFraction(double drain, double onset, double floor) {
    if (drain < kDrainOnsetAt) return 1.0 - (1.0 - onset) * drain / kDrainOnsetAt;
    return onset - (onset - floor) * (drain - kDrainOnsetAt) / (1.0 - kDrainOnsetAt);
}

// Fills ports and leakage inputs for a list of transistors at a temperature.
template <int P, int I>
inline void setDevices(const Device* devices, int count, double temperatureC, PortModel<P>& ports, double u[I]) {
    for (int q = 0; q < count; ++q) {
        const Device& d = devices[q];
        double bandgap = d.germanium ? kGermaniumBandgap : kSiliconBandgap;
        double is = d.saturation * saturationScale(temperatureC, d.emission, bandgap);
        ports.setTransistor(2 * q, is, d.emission, d.betaF, d.betaR, temperatureC);
        if (d.germanium && INPUT_LEAK + q < I - 1) u[INPUT_LEAK + q] = leakageAt(d.leakage, temperatureC);
    }
}

// --------------------------------------------------------------------------
// A model that is one stage. Circuit supplies the netlist and constants:
//   NODES, STATES, PORTS, INPUTS, TRANSISTORS
//   static void describe(const ModelControls&, Netlist&, int& outputNode)
//   static const Device* devices()
//   railVolts(), drainFloor(), drainHeat(), noiseAmps()
// --------------------------------------------------------------------------
template <class Circuit>
struct SingleStageModel : FuzzModel {
    Stage<Circuit::NODES, Circuit::STATES, Circuit::PORTS, Circuit::INPUTS> stage;
    double builtFuzz = -1.0, builtGuitar = -1.0, builtDrain = -1.0;

    bool configure(const ModelControls& c) override {
        // Nothing moved since the last block: netlists, ports and inputs stand.
        bool moved = std::fabs(c.fuzz - builtFuzz) > 1e-4 || std::fabs(c.guitar - builtGuitar) > 1e-4;
        if (!moved && c.drain == builtDrain) return false;
        builtDrain = c.drain;
        stage.net.count = 0;
        Circuit::describe(c, stage.net, stage.output);
        double fraction = railFraction(c.drain, Circuit::drainOnset(), Circuit::drainFloor());
        double temperature = drainTemperature(fraction, Circuit::drainHeat());
        setDevices<Circuit::PORTS, Circuit::INPUTS>(Circuit::devices(), Circuit::TRANSISTORS, temperature, stage.ports, stage.u);
        stage.u[INPUT_RAIL] = Circuit::railVolts() * fraction;
        if (moved) { builtFuzz = c.fuzz; builtGuitar = c.guitar; }
        return moved;
    }
    bool start(double T, bool warm) override { return stage.start(T, 0.0, warm); }
    void rebuild(double T) override { stage.rebuild(T); }
    void refresh() override { stage.refresh(); }
    void prepare(double noise) override { stage.core.prepare(noise); }
    double step(double emf) override { return stage.core.step(emf) - stage.rest; }
    bool finite() const override { return stage.core.finite(); }
    double noiseScale() const override { return Circuit::noiseAmps(); }
    int deviceCount() const override { return Circuit::TRANSISTORS; }
    DeviceKind deviceKind(int) const override { return Circuit::railVolts() < 0.0 ? DEVICE_PNP : DEVICE_NPN; }
    void sense(float forward[kMaxDevices], float reverse[kMaxDevices]) const override {
        for (int q = 0; q < Circuit::TRANSISTORS; ++q) {
            forward[q] = (float)stage.core.current[2 * q];
            reverse[q] = (float)stage.core.current[2 * q + 1];
        }
    }
};

// --------------------------------------------------------------------------
// Fuzz Face, germanium (Arbiter, 1966). Q1 into Q2 direct, R2 feeding Q2's
// emitter back to Q1's base, FUZZ the 1K pot bypassed by 20 uF.
// Trimmer values are a tech's bias at 2 uA leakage (TP2 at 4.5 V).
// --------------------------------------------------------------------------
struct FaceCircuit {
    enum { M, P, W, B1, B1I, X, XI, E2, RW, Y, Z, RAIL, NODES };
    static const int STATES = 6, TRANSISTORS = 2, PORTS = 4, INPUTS = TRANSISTORS + 3;
    static double railVolts() { return -9.0; }
    // Battery fractions, measured on a riff: onset where the output is 3 dB
    // down, floor (BATTERY at 0) just past where it dies (45 dB down). The
    // junctions warm by drainHeat at the bottom of the knob.
    static double drainOnset() { return 0.62; }
    static double drainFloor() { return 0.04; }
    static double drainHeat()  { return 30.0; }
    static double noiseAmps()  { return 1.0; }
    static Device* devices() {
        static Device d[2] = { { true, 1e-6, 1.1, 80.0, 5.0, 2e-6 }, { true, 1e-6, 1.1, 120.0, 5.0, 2e-6 } };
        return d;
    }
    // FUZZ: the 1K pot's wiper, a measured reverse-audio taper so each tenth
    // of the knob brightens the fuzz by the same ratio.
    static void describe(const ModelControls& c, Netlist& net, int& output) {
        static const double wiper[11] = { 0.0, 0.30, 0.50, 0.63, 0.71, 0.78, 0.83, 0.88, 0.92, 0.96, 1.0 };
        double fuzz = lookup11(wiper, c.fuzz);
        addPickup(net, M, P, W, c.guitar);
        addRail(net, RAIL);
        net.cap(W, B1, 2.2e-6);
        net.res(B1, E2, 100e3);
        net.res(X, RAIL, 20124.0);                // Trimmer1
        net.res(E2, RW, (1.0 - fuzz) * 1000.0);
        net.res(RW, GND, fuzz * 1000.0);
        net.cap(RW, GND, 20e-6);
        net.res(Y, Z, 8530.0);                    // Trimmer2
        net.res(Z, RAIL, 470.0);
        // Each with the classic 100 pF base-collector snubber, fixed.
        addGermaniumPnp(net, GND, B1, B1I, X, devices()[0], 0, INPUT_LEAK + 0, 100e-12);
        addGermaniumPnp(net, E2, X, XI, Y, devices()[1], 2, INPUT_LEAK + 1, 100e-12);
        net.isrc(B1I, 1.0, INPUTS - 1);           // noise into Q1's base
        output = Z;
    }
};

// --------------------------------------------------------------------------
// Tone Bender MK1 (Sola Sound, 1965). OC75 emitter follower, then two
// 2G381 common-emitter stages. ATTACK is Q2's base bias: its resistance to
// ground against the 470K to the rail.
// --------------------------------------------------------------------------
struct BenderMk1Circuit {
    enum { M, P, W, B1, B1I, E1, B2, B2I, ATT, C2, B3, B3I, C3, RAIL, NODES };
    static const int STATES = 8, TRANSISTORS = 3, PORTS = 6, INPUTS = TRANSISTORS + 3;
    static double railVolts() { return -9.0; }
    static double drainOnset() { return 0.62; }
    static double drainFloor() { return 0.32; }
    static double drainHeat()  { return 30.0; }
    static double noiseAmps()  { return 1.0; }
    static Device* devices() {
        // Leakage picked so Q2 and Q3 sit at -4.5 V with FUZZ centred.
        static Device d[3] = { { true, 1e-6, 1.1, 90.0, 5.0, 4e-6 }, { true, 1e-6, 1.1, 80.0, 5.0, 22.7e-6 },
                               { true, 1e-6, 1.1, 80.0, 5.0, 22.3e-6 } };
        return d;
    }
    // FUZZ: ATTACK's resistance to ground, ohms, for knob 0 .. 1. Measured:
    // below ~1.2K Q2 starves (sputter, the job of DRAIN); above ~20K nothing
    // changes. Logarithmic between.
    static void describe(const ModelControls& c, Netlist& net, int& output) {
        static const double attack[11] = { 1200, 1590, 2106, 2791, 3698, 4899, 6491, 8600, 11394, 15095, 20000 };
        addPickup(net, M, P, W, c.guitar);
        addRail(net, RAIL);
        net.cap(W, B1, 0.01e-6);
        net.res(B1, GND, 1e6);
        net.res(E1, GND, 8200.0);
        net.cap(E1, B2, 25e-6);
        net.res(B2, RAIL, 470e3);
        net.res(B2, ATT, 2200.0);
        net.res(ATT, GND, lookup11(attack, c.fuzz));
        net.res(C2, RAIL, 2200.0);
        net.cap(C2, B3, 0.1e-6);
        net.res(B3, GND, 8200.0);
        net.res(C3, RAIL, 15e3);
        addGermaniumPnp(net, E1, B1, B1I, RAIL, devices()[0], 0, INPUT_LEAK + 0);
        addGermaniumPnp(net, GND, B2, B2I, C2, devices()[1], 2, INPUT_LEAK + 1);
        addGermaniumPnp(net, GND, B3, B3I, C3, devices()[2], 4, INPUT_LEAK + 2);
        net.isrc(B2I, 1.0, INPUTS - 1);
        output = C3;
    }
};

// --------------------------------------------------------------------------
// Tone Bender MK2 (Sola Sound, 1966). An OC75 boost stage direct-coupled
// into the Fuzz-Face-like pair, 100K feedback from Q3's emitter to Q2's
// base, ATTACK the 1K emitter pot bypassed by 5 uF. Output from the top of
// the 8.2K load, as drawn.
// --------------------------------------------------------------------------
struct BenderMk2Circuit {
    enum { M, P, W, B1, B1I, X1, B2I, C2, B3I, E3, ATTW, C3, T, RAIL, NODES };
    static const int STATES = 8, TRANSISTORS = 3, PORTS = 6, INPUTS = TRANSISTORS + 3;
    static double railVolts() { return -9.0; }
    static double drainOnset() { return 0.92; }
    static double drainFloor() { return 0.60; }
    static double drainHeat()  { return 30.0; }
    // Three gain stages: full-strength junction noise hisses at -35 dB under
    // the signal at full FUZZ. Hand-tunable.
    static double noiseAmps()  { return 0.3; }
    static Device* devices() {
        // Q1's leakage is its bias (its base returns to ground through 100K);
        // picked so Q3's collector sits at -4.5 V, the MK2 bias point.
        static Device d[3] = { { true, 1e-6, 1.1, 70.0, 5.0, 15.0e-6 }, { true, 1e-6, 1.1, 90.0, 5.0, 3e-6 },
                               { true, 1e-6, 1.1, 110.0, 5.0, 3e-6 } };
        return d;
    }
    static void describe(const ModelControls& c, Netlist& net, int& output) {
        static const double wiper[11] = { 0.0, 0.30, 0.50, 0.63, 0.71, 0.78, 0.83, 0.88, 0.92, 0.96, 1.0 };
        double attack = lookup11(wiper, c.fuzz);
        addPickup(net, M, P, W, c.guitar);
        addRail(net, RAIL);
        net.cap(W, GND, 0.01e-6);
        net.cap(W, B1, 5e-6);
        net.res(B1, GND, 100e3);
        net.res(X1, RAIL, 10e3);
        net.res(C2, RAIL, 33e3);
        net.res(C3, T, 8200.0);
        net.res(T, RAIL, 470.0);
        net.res(X1, E3, 100e3);
        net.res(E3, ATTW, (1.0 - attack) * 1000.0);
        net.res(ATTW, GND, attack * 1000.0 + 1.0);
        net.cap(ATTW, GND, 5e-6);
        addGermaniumPnp(net, GND, B1, B1I, X1, devices()[0], 0, INPUT_LEAK + 0);
        addGermaniumPnp(net, GND, X1, B2I, C2, devices()[1], 2, INPUT_LEAK + 1);
        addGermaniumPnp(net, E3, C2, B3I, C3, devices()[2], 4, INPUT_LEAK + 2);
        net.isrc(B1I, 1.0, INPUTS - 1);
        output = T;
    }
};

// --------------------------------------------------------------------------
// Vox Tone Bender (made by JEN, Italy). The Fuzz Face pair with 47K
// feedback, a 1K ATTACK pot over an 8.2K emitter resistor bypassed by
// 15 uF, and the output from the top of the 8.2K load.
// --------------------------------------------------------------------------
struct VoxCircuit {
    enum { M, P, W, B1, B1I, X, XI, E2, ATTW, N8, C2, T, RAIL, NODES };
    static const int STATES = 6, TRANSISTORS = 2, PORTS = 4, INPUTS = TRANSISTORS + 3;
    static double railVolts() { return -9.0; }
    static double drainOnset() { return 0.48; }
    static double drainFloor() { return 0.03; }
    static double drainHeat()  { return 30.0; }
    static double noiseAmps()  { return 1.0; }
    static Device* devices() {
        static Device d[2] = { { true, 1e-6, 1.1, 90.0, 5.0, 2e-6 }, { true, 1e-6, 1.1, 110.0, 5.0, 2e-6 } };
        return d;
    }
    static void describe(const ModelControls& c, Netlist& net, int& output) {
        static const double wiper[11] = { 0.0, 0.30, 0.50, 0.63, 0.71, 0.78, 0.83, 0.88, 0.92, 0.96, 1.0 };
        double attack = lookup11(wiper, c.fuzz);
        addPickup(net, M, P, W, c.guitar);
        addRail(net, RAIL);
        net.cap(W, B1, 0.15e-6);
        net.res(B1, E2, 47e3);
        net.res(X, RAIL, 10e3);
        net.res(C2, T, 8200.0);
        net.res(T, RAIL, 1000.0);
        net.res(E2, ATTW, (1.0 - attack) * 1000.0);
        net.res(ATTW, N8, attack * 1000.0);
        net.res(N8, GND, 8200.0);
        net.cap(ATTW, GND, 15e-6);
        addGermaniumPnp(net, GND, B1, B1I, X, devices()[0], 0, INPUT_LEAK + 0);
        addGermaniumPnp(net, E2, X, XI, C2, devices()[1], 2, INPUT_LEAK + 1);
        net.isrc(B1I, 1.0, INPUTS - 1);
        output = T;
    }
};

// --------------------------------------------------------------------------
// Maestro FZ-1 Fuzz-Tone (Gibson, 1962), 3 V version. A 2N270 follower,
// then two common-emitter stages. Q2's base has no bias from the rail: it
// runs on its own leakage, and FUZZ shunts that to ground, which is the
// sputter of the original.
// --------------------------------------------------------------------------
struct MaestroCircuit {
    enum { M, P, W, N1, B1, B1I, E1, B2, B2I, BB, C2, B3, B3I, C3, RAIL, NODES };
    static const int STATES = 8, TRANSISTORS = 3, PORTS = 6, INPUTS = TRANSISTORS + 3;
    static double railVolts() { return -3.0; }
    static double drainOnset() { return 0.82; }
    static double drainFloor() { return 0.65; }
    static double drainHeat()  { return 25.0; }
    static double noiseAmps()  { return 1.0; }
    static Device* devices() {
        // Q2 and Q3 run on leakage alone; picked so both collectors sit at
        // -1.5 V (half the 3 V rail) with FUZZ centred.
        static Device d[3] = { { true, 1e-6, 1.1, 70.0, 5.0, 6e-6 }, { true, 1e-6, 1.1, 70.0, 5.0, 33.6e-6 },
                               { true, 1e-6, 1.1, 70.0, 5.0, 17.7e-6 } };
        return d;
    }
    // FUZZ: the 50K pot's resistance to ground, ohms, in parallel with 22K.
    // Measured sweet spot 1.2K (sputtery, the original's starved edge) to
    // 20K; logarithmic between.
    static void describe(const ModelControls& c, Netlist& net, int& output) {
        static const double fuzzOhms[11] = { 1200, 1590, 2106, 2791, 3698, 4899, 6491, 8600, 11394, 15095, 20000 };
        addPickup(net, M, P, W, c.guitar);
        addRail(net, RAIL);
        net.res(W, N1, 100e3);
        net.cap(N1, B1, 0.01e-6);
        net.res(B1, GND, 1e6);
        net.res(E1, GND, 10e3);
        net.cap(E1, B2, 20e-6);
        net.res(B2, BB, 2200.0);
        net.res(BB, GND, 22e3);
        net.res(BB, GND, lookup11(fuzzOhms, c.fuzz));
        net.res(C2, RAIL, 1500.0);
        net.cap(C2, B3, 20e-6);
        net.res(B3, GND, 56e3);
        net.res(B3, GND, 10e3);
        net.res(C3, RAIL, 10e3);
        addGermaniumPnp(net, E1, B1, B1I, RAIL, devices()[0], 0, INPUT_LEAK + 0);
        addGermaniumPnp(net, GND, B2, B2I, C2, devices()[1], 2, INPUT_LEAK + 1);
        addGermaniumPnp(net, GND, B3, B3I, C3, devices()[2], 4, INPUT_LEAK + 2);
        net.isrc(B2I, 1.0, INPUTS - 1);
        output = C3;
    }
};

// --------------------------------------------------------------------------
// Colorsound Power Boost, early 18 V version. Q1 Q2 a direct-coupled
// silicon pair (Q1's collector load is Q2's base bias, 150K from the tap in
// Q2's emitter leg back to Q1's base) with series feedback DC-coupled from
// Q2's collector through 12K straight into Q1's emitter (4K7 to ground),
// where the 10K VOLUME rheostat behind 22 uF sets the gain, 1 + 12K / DRIVE.
// Q2's emitter is 470 + 1K2 with 22 uF across the whole leg; the 220 pF
// Miller cap rolls off the top. Then an active Baxandall: Q3's base takes
// the network's centre, its collector drives the network's far end through
// 22 uF; BASS and TREBLE sit at noon. Q3 runs 180K / 33K, 3K9 and a 1K
// emitter bypassed by 4.7 uF. One stage: the tone network loads the
// feedback node, so the two cannot be cut apart cleanly.
// --------------------------------------------------------------------------
struct PowerBoostCircuit {
    enum { M, P, W, B1, C1, E1, VR, E2, TAP, C2, X, BL, BR, BW, TM, TW, TL, TR, Y, B3, C3, E3, RAIL, NODES };
    static const int STATES = 14, TRANSISTORS = 3, PORTS = 6, INPUTS = TRANSISTORS + 3;
    static double railVolts() { return 18.0; }
    static double drainOnset() { return 0.72; }
    static double drainFloor() { return 0.11; }
    static double drainHeat()  { return 10.0; }
    static double noiseAmps()  { return 0.3; }
    static Device* devices() {
        static Device d[3] = { { false, 2e-14, 1.0, 450.0, 5.0, 0.0 }, { false, 2e-14, 1.0, 450.0, 5.0, 0.0 },
                               { false, 2e-14, 1.0, 300.0, 5.0, 0.0 } };
        return d;
    }
    static constexpr double kBass = 0.5, kTreble = 0.5;
    // FUZZ: the VOLUME rheostat's resistance, ohms, for knob 0 .. 1. Logarithmic,
    // measured so each tenth of the knob is audible; above ~300 ohms it is a clean boost.
    static void describe(const ModelControls& c, Netlist& net, int& output) {
        static const double drive[11] = { 300, 200, 140, 100, 70, 50, 36, 26, 18, 12, 8 };
        addPickup(net, M, P, W, c.guitar);
        addRail(net, RAIL);
        net.cap(W, B1, 0.22e-6);
        net.res(B1, TAP, 150e3);
        net.res(C1, RAIL, 120e3);
        net.bjt(E1, B1, C1, false, devices()[0].betaF, devices()[0].betaR, 0);
        // Q1's base-collector capacitance with its stray. Without it the
        // feedback loop's fastest pole sits beyond what 4x resolves and rings
        // into broadband junk; at 22 pF the loop settles. Hand-tunable, 10 pF upward.
        net.cap(B1, C1, 22e-12);
        net.res(E1, GND, 4700.0);
        net.cap(E1, VR, 22e-6);
        net.res(VR, GND, lookup11(drive, c.fuzz));
        net.res(C2, E1, 12e3);
        net.bjt(E2, C1, C2, false, devices()[1].betaF, devices()[1].betaR, 2);
        net.cap(C1, C2, 220e-12);
        net.res(E2, TAP, 470.0);
        net.res(TAP, GND, 1200.0);
        net.cap(E2, GND, 22e-6);
        net.res(C2, RAIL, 1800.0);
        net.cap(C2, X, 4.7e-6);
        net.res(X, BL, 4700.0);
        net.res(BL, BW, kBass * 100e3);
        net.res(BW, BR, (1.0 - kBass) * 100e3);
        net.cap(BL, BR, 0.1e-6);
        net.res(BR, Y, 4700.0);
        net.res(BW, TM, 39e3);
        net.res(TM, TW, 5600.0);
        net.cap(X, TL, 0.01e-6);
        net.res(TL, TW, kTreble * 100e3);
        net.res(TW, TR, (1.0 - kTreble) * 100e3);
        net.cap(TR, Y, 0.01e-6);
        net.cap(TM, B3, 0.1e-6);
        net.res(B3, RAIL, 180e3);
        net.res(B3, GND, 33e3);
        net.bjt(E3, B3, C3, false, devices()[2].betaF, devices()[2].betaR, 4);
        net.res(E3, GND, 1000.0);
        net.cap(E3, GND, 4.7e-6);
        net.res(C3, RAIL, 3900.0);
        net.cap(Y, C3, 22e-6);
        net.isrc(B1, 1.0, INPUTS - 1);
        output = C3;
    }
};

// --------------------------------------------------------------------------
// Univox Super-Fuzz (Shin-ei, ~1968), tone switch on the scooped setting.
// Solved as three stages, each handing its output voltage to the next:
//   A  input pair Q1 Q2 with its feedback network, EXPANDER, and Q3's bias
//      network as the load
//   B  Q3 phase splitter into the Q4 Q5 octave pair (common collector)
//   C  the octave output (a 10K Thevenin source) into the OA90 pair, the
//      scoop network, BALANCE, and Q6's bias network as the load
// The cuts sit where a stage only sees the next one's light base load.
// Stage A never clips (Q1 and Q2 stay out of saturation; its distortion is
// low-order), so it runs at the base rate in front of the upsampler: the
// octave and the diodes behind it run oversampled. Measured against stage A
// at 4x: band levels within 0.02 dB, aliasing unchanged.
// Q6, the output stage, never clips: the diodes hold its input under
// ~80 mV pp, where it is a plain inverting gain of 10K / (1K + re), so it
// runs as that gain (measured identical to the full solve).
// --------------------------------------------------------------------------
struct SuperFuzzModel : FuzzModel {
    struct StageA { enum { M, P, W, N1, B1, F, C1, E1, E2, G, EXT, WIP, H, RAIL, NODES }; };
    struct StageB { enum { HN, C3, E3, J1, J, K1, K, L, EO, RAIL, NODES }; };
    struct StageC { enum { LN, D, NN, S, R, BW, QB, RAIL, NODES }; };
    Stage<StageA::NODES, 9, 4, 3> stageA;
    Stage<StageB::NODES, 5, 6, 3> stageB;
    Stage<StageC::NODES, 5, 2, 3> stageC;
    double builtFuzz = -1.0, builtGuitar = -1.0, builtDrain = -1.0;

    // Balance, fixed. Hand-tunable 0..1.
    static constexpr double kBalance = 0.7;
    // Silicon shrugs off heat, so the drain is the battery alone. Fractions
    // of 9 V as for the single-stage circuits: 3 dB down, and just dead.
    static constexpr double kDrainOnset = 0.42, kDrainFloor = 0.32;
    // Drives between stages: an ideal-ish source into a node.
    static constexpr double kDriveOhms = 1.0;
    // Q6: -Rc / (Re + re), re = Vt / 0.49 mA.
    static constexpr double kOutputGain = -10e3 / (1000.0 + 52.0);

    bool configure(const ModelControls& c) override {
        // Nothing moved since the last block: netlists, ports and inputs stand.
        bool moved = std::fabs(c.fuzz - builtFuzz) > 1e-4 || std::fabs(c.guitar - builtGuitar) > 1e-4;
        if (!moved && c.drain == builtDrain) return false;
        builtDrain = c.drain;
        static const double expander[11] = { 0.05, 0.10, 0.16, 0.23, 0.31, 0.40, 0.50, 0.61, 0.73, 0.86, 1.0 };
        double x = lookup11(expander, c.fuzz);
        const double b = kBalance;
        {
            Netlist& net = stageA.net; net.count = 0;
            typedef StageA A;
            addPickup(net, A::M, A::P, A::W, c.guitar);
            addRail(net, A::RAIL);
            net.cap(A::W, A::N1, 10e-6);
            net.res(A::N1, A::B1, 22e3);
            net.res(A::B1, GND, 100e3);
            net.res(A::B1, A::F, 100e3);
            net.cap(A::F, GND, 0.1e-6);
            net.res(A::F, A::E2, 470e3);
            net.res(A::F, A::G, 47e3);
            net.cap(A::G, A::E2, 10e-6);
            net.res(A::E1, GND, 1800.0);
            net.res(A::C1, A::RAIL, 47e3);
            net.cap(A::RAIL, A::C1, 0.001e-6);
            net.cap(A::B1, A::C1, kSiliconCjc);
            net.bjt(A::E1, A::B1, A::C1, false, kSilicon.betaF, kSilicon.betaR, 0);
            net.bjt(A::E2, A::C1, A::RAIL, false, kSilicon.betaF, kSilicon.betaR, 2);
            net.res(A::E2, GND, 10e3);
            net.cap(A::E2, A::EXT, 10e-6);
            net.res(A::EXT, A::WIP, (1.0 - x) * 50e3);
            net.res(A::WIP, GND, x * 50e3);
            net.cap(A::WIP, A::H, 10e-6);
            net.res(A::H, A::RAIL, 220e3);
            net.res(A::H, GND, 150e3);
            net.isrc(A::B1, 1.0, 2);
            stageA.output = A::H;
        }
        {
            Netlist& net = stageB.net; net.count = 0;
            typedef StageB B;
            addDrive(net, B::HN, kDriveOhms);
            addRail(net, B::RAIL);
            net.bjt(B::E3, B::HN, B::C3, false, kSilicon.betaF, kSilicon.betaR, 0);
            net.res(B::C3, B::RAIL, 10e3);
            net.res(B::E3, GND, 10e3);
            net.cap(B::C3, B::J1, 10e-6);
            net.res(B::J1, B::J, 470.0);
            net.res(B::J, B::RAIL, 100e3);
            net.res(B::J, GND, 22e3);
            net.cap(B::E3, B::K1, 10e-6);
            net.res(B::K1, B::K, 470.0);
            net.res(B::K, B::RAIL, 100e3);
            net.res(B::K, GND, 22e3);
            net.bjt(B::EO, B::J, B::L, false, kSilicon.betaF, kSilicon.betaR, 2);
            net.bjt(B::EO, B::K, B::L, false, kSilicon.betaF, kSilicon.betaR, 4);
            net.cap(B::J, B::L, kSiliconCjc);
            net.cap(B::K, B::L, kSiliconCjc);
            net.res(B::L, B::RAIL, 10e3);
            net.res(B::EO, GND, 1800.0);
            net.cap(B::EO, GND, 10e-6);
            net.isrc(B::J, 1.0, 2);
            stageB.output = B::L;
        }
        {
            Netlist& net = stageC.net; net.count = 0;
            typedef StageC C;
            addDrive(net, C::LN, 10e3);           // the octave pair's 10K collector load
            addRail(net, C::RAIL);
            net.cap(C::LN, C::D, 10e-6);
            net.diode(C::D, GND, 0);
            net.diode(GND, C::D, 1);
            net.cap(C::D, C::NN, 10e-6);
            net.res(C::NN, GND, 57e3);            // the flat branch (47K + 10K), unused but loading
            net.cap(C::NN, C::S, 0.001e-6);
            net.res(C::NN, C::R, 22e3);
            net.res(C::S, C::R, 10e3);
            net.cap(C::R, GND, 0.1e-6);
            net.res(C::S, C::BW, (1.0 - b) * 50e3);
            net.res(C::BW, GND, b * 50e3);
            net.cap(C::BW, C::QB, 10e-6);
            net.res(C::QB, C::RAIL, 100e3);
            net.res(C::QB, GND, 15e3);
            net.res(C::QB, GND, 210e3);           // Q6's input, ~beta x 1K
            net.isrc(C::D, 1.0, 2);
            stageC.output = C::QB;
        }
        // Silicon barely cares about heat; the drain is the battery.
        const double fraction = railFraction(c.drain, kDrainOnset, kDrainFloor);
        const double temperature = drainTemperature(fraction, 10.0);
        const double rail = 9.0 * fraction;
        Device si[3] = { kSilicon, kSilicon, kSilicon };
        setDevices<4, 3>(si, 2, temperature, stageA.ports, stageA.u);
        setDevices<6, 3>(si, 3, temperature, stageB.ports, stageB.u);
        stageC.ports.setDiode(0, kDiodeSaturation * saturationScale(temperature, kDiodeEmission, kGermaniumBandgap), kDiodeEmission, temperature);
        stageC.ports.setDiode(1, kDiodeSaturation * saturationScale(temperature, kDiodeEmission, kGermaniumBandgap), kDiodeEmission, temperature);
        stageA.u[INPUT_RAIL] = stageB.u[INPUT_RAIL] = stageC.u[INPUT_RAIL] = rail;
        if (moved) { builtFuzz = c.fuzz; builtGuitar = c.guitar; }
        return moved;
    }
    bool start(double T, bool warm) override {
        // Each stage settles on the one before it's resting output.
        if (!stageA.start(T * kOversample, 0.0, warm)) return false;
        if (!stageB.start(T, stageA.rest, warm)) return false;
        return stageC.start(T, stageB.rest, warm);
    }
    void rebuild(double T) override { stageA.rebuild(T * kOversample); stageB.rebuild(T); stageC.rebuild(T); }
    void refresh() override { stageA.refresh(); stageB.refresh(); stageC.refresh(); }
    void prepare(double noise) override {
        stageA.core.prepare(noise);
        stageB.core.prepare(0.0);
        stageC.core.prepare(0.0);
    }
    // Stage A's swing around its rest point, upsampled by the caller.
    double front(double emf) override { return stageA.core.step(emf) - stageA.rest; }
    double step(double swing) override {
        double b = stageB.core.step(swing + stageA.rest);
        double c = stageC.core.step(b);
        return kOutputGain * (c - stageC.rest);
    }
    bool finite() const override {
        return stageA.core.finite() && stageB.core.finite() && stageC.core.finite();
    }
    double noiseScale() const override { return 0.3; }
    // Q1 Q2 (stage A), Q3 Q4 Q5 (stage B), the diode pair (stage C).
    int deviceCount() const override { return 7; }
    DeviceKind deviceKind(int index) const override { return index < 5 ? DEVICE_NPN : DEVICE_DIODE; }
    void sense(float forward[kMaxDevices], float reverse[kMaxDevices]) const override {
        for (int q = 0; q < 2; ++q) { forward[q] = (float)stageA.core.current[2 * q]; reverse[q] = (float)stageA.core.current[2 * q + 1]; }
        for (int q = 0; q < 3; ++q) { forward[2 + q] = (float)stageB.core.current[2 * q]; reverse[2 + q] = (float)stageB.core.current[2 * q + 1]; }
        for (int d = 0; d < 2; ++d) { forward[5 + d] = (float)stageC.core.current[d]; reverse[5 + d] = 0.f; }
    }
};

// --------------------------------------------------------------------------
// JFET-input octave fuzz (the 1970s Japanese "standard fuzz" of the
// Shin-ei family): a 2SK30 common-source stage, FUZZ DEPTH (a 50K pot into
// the phase splitter), Q2 phase splitter into the Q3 Q4 octave pair, the
// germanium diode pair, and the tone switch on its flat setting (the R15 R16
// divider), BALANCE, Q5 output stage. Solved as:
//   front  (base rate) the pickup into the gate, then the JFET and its
//          bypassed source resistor in closed form; the drain is handed on
//          as a Thevenin source (rail - Rd Id behind Rd)
//   B      FUZZ DEPTH, Q2 (collector-feedback bias), the octave pair
//   C      the octave collector (a 10K Thevenin source) into the diodes,
//          the flat tap, BALANCE and Q5's bias network
// Q5 runs as its gain, as in the Super-Fuzz model.
// --------------------------------------------------------------------------
struct FetOctaveModel : FuzzModel {
    struct StageG { enum { M, P, W, G, NODES }; };
    struct StageB { enum { DR, PT, WP, B2, X, EM, X1, B3, E1, B4, Y, EO, RAIL, NODES }; };
    struct StageC { enum { YN, D, N, F, BW, QB, RAIL, NODES }; };
    Stage<StageG::NODES, 3, 1, 3> stageG;
    Stage<StageB::NODES, 8, 6, 3> stageB;
    Stage<StageC::NODES, 3, 2, 3> stageC;
    double builtFuzz = -1.0, builtGuitar = -1.0, builtDrain = -1.0;

    // 2SK30A-Y, square law. Idss and Vp picked inside the grade's range so
    // the bias lands where the schematic marks it (source 0.9 V, drain
    // 5.98 V, 0.3 mA). Hand-tunable.
    static constexpr double kIdss = 2.0e-3, kPinchOff = -1.47;
    static constexpr double kDrainOhms = 10e3, kSourceOhms = 3e3, kSourceFarads = 10e-6;
    // Balance, fixed. Hand-tunable 0..1.
    static constexpr double kBalance = 0.7;
    // Battery fractions of 9 V: 3 dB down, and just dead (measured).
    static constexpr double kDrainOnset = 0.45, kDrainFloor = 0.12;
    // Q5: -Rc / (Re + re), re = Vt / 0.56 mA.
    static constexpr double kOutputGain = -10e3 / (1000.0 + 46.0);

    // JFET state, base rate.
    double sourceVolts = 0.0, drainAmps = 0.0, frontRest = 0.0;
    double railTarget = 9.0, rail = 9.0, baseStep = 1.0 / 48000.0;

    // Drain current for gate voltage `gate`, advancing the source node one
    // trapezoidal step: Id = k (Vg - Vs - Vp)^2, solved in closed form.
    double jfetStep(double gate) {
        const double k = kIdss / (kPinchOff * kPinchOff);
        const double h = baseStep / (2.0 * kSourceFarads);
        const double b = 1.0 + h / kSourceOhms;
        const double q = sourceVolts * (1.0 - h / kSourceOhms) + h * drainAmps;
        const double headroom = gate - kPinchOff;            // Vs at which the channel pinches off
        const double c = headroom * b - q;
        double overdrive = (c > 0.0) ? (-b + std::sqrt(b * b + 4.0 * h * k * c)) / (2.0 * h * k) : 0.0;
        if (overdrive > 0.0) { drainAmps = k * overdrive * overdrive; sourceVolts = headroom - overdrive; }
        else                 { drainAmps = 0.0;                       sourceVolts = q / b; }
        return drainAmps;
    }
    // The JFET at rest with the gate at 0 V.
    void jfetRest() {
        const double k = kIdss / (kPinchOff * kPinchOff), rk = kSourceOhms * k;
        double overdrive = (-1.0 + std::sqrt(1.0 - 4.0 * rk * kPinchOff)) / (2.0 * rk);
        drainAmps = k * overdrive * overdrive;
        sourceVolts = -kPinchOff - overdrive;
    }

    bool configure(const ModelControls& c) override {
        // Nothing moved since the last block: netlists, ports and inputs stand.
        bool moved = std::fabs(c.fuzz - builtFuzz) > 1e-4 || std::fabs(c.guitar - builtGuitar) > 1e-4;
        if (!moved && c.drain == builtDrain) return false;
        builtDrain = c.drain;
        // FUZZ DEPTH wiper, measured so each tenth of the knob is audible.
        static const double depth[11] = { 0.03, 0.06, 0.10, 0.15, 0.21, 0.28, 0.37, 0.48, 0.61, 0.78, 1.0 };
        double wiper = lookup11(depth, c.fuzz);
        const double b = kBalance;
        {
            Netlist& net = stageG.net; net.count = 0;
            typedef StageG G;
            addPickup(net, G::M, G::P, G::W, c.guitar);
            net.cap(G::W, G::G, 10e-6);
            net.res(G::G, GND, 100e3);
            net.diode(G::G, GND, 0);              // the gate junction, reverse biased, for the solver's port
            net.isrc(G::G, 1.0, 2);
            stageG.output = G::G;
        }
        {
            Netlist& net = stageB.net; net.count = 0;
            typedef StageB B;
            addDrive(net, B::DR, kDrainOhms);       // Q1's drain behind Rd
            addRail(net, B::RAIL);
            net.cap(B::DR, B::PT, 10e-6);
            net.res(B::PT, B::WP, (1.0 - wiper) * 50e3);
            net.res(B::WP, GND, wiper * 50e3);
            net.cap(B::WP, B::B2, 10e-6);
            net.res(B::B2, B::X, 510e3);
            net.res(B::X, B::RAIL, 10e3);
            net.res(B::EM, GND, 10e3);
            net.bjt(B::EM, B::B2, B::X, false, kSilicon.betaF, kSilicon.betaR, 0);
            net.cap(B::B2, B::X, kSiliconCjc);
            net.cap(B::X, B::X1, 10e-6);
            net.res(B::X1, B::B3, 1000.0);
            net.res(B::B3, B::RAIL, 150e3);
            net.res(B::B3, GND, 22e3);
            net.cap(B::EM, B::E1, 10e-6);
            net.res(B::E1, B::B4, 1000.0);
            net.res(B::B4, B::RAIL, 150e3);
            net.res(B::B4, GND, 22e3);
            net.bjt(B::EO, B::B3, B::Y, false, kSilicon.betaF, kSilicon.betaR, 2);
            net.bjt(B::EO, B::B4, B::Y, false, kSilicon.betaF, kSilicon.betaR, 4);
            net.cap(B::B3, B::Y, kSiliconCjc);
            net.cap(B::B4, B::Y, kSiliconCjc);
            net.res(B::Y, B::RAIL, 10e3);
            net.res(B::EO, GND, 1000.0);
            net.cap(B::EO, GND, 10e-6);
            stageB.output = B::Y;
        }
        {
            Netlist& net = stageC.net; net.count = 0;
            typedef StageC C;
            addDrive(net, C::YN, 10e3);           // the octave pair's 10K collector load
            addRail(net, C::RAIL);
            net.cap(C::YN, C::D, 10e-6);
            net.diode(C::D, GND, 0);
            net.diode(GND, C::D, 1);
            net.cap(C::D, C::N, 10e-6);
            net.res(C::N, C::F, 47e3);
            net.res(C::F, GND, 10e3);
            net.res(C::N, GND, 22e3);             // the unused scoop network's load: R17 into C11, near ground in band
            net.res(C::F, C::BW, (1.0 - b) * 50e3);
            net.res(C::BW, GND, b * 50e3);
            net.cap(C::BW, C::QB, 10e-6);
            net.res(C::QB, C::RAIL, 100e3);
            net.res(C::QB, GND, 15e3);
            net.res(C::QB, GND, 210e3);           // Q5's input, ~beta x 1K
            stageC.output = C::QB;
        }
        const double fraction = railFraction(c.drain, kDrainOnset, kDrainFloor);
        const double temperature = drainTemperature(fraction, 10.0);
        railTarget = 9.0 * fraction;
        Device si[3] = { kSilicon, kSilicon, kSilicon };
        stageG.ports.setDiode(0, 1e-16, 1.0, temperature);
        setDevices<6, 3>(si, 3, temperature, stageB.ports, stageB.u);
        stageC.ports.setDiode(0, kDiodeSaturation * saturationScale(temperature, kDiodeEmission, kGermaniumBandgap), kDiodeEmission, temperature);
        stageC.ports.setDiode(1, kDiodeSaturation * saturationScale(temperature, kDiodeEmission, kGermaniumBandgap), kDiodeEmission, temperature);
        stageB.u[INPUT_RAIL] = stageC.u[INPUT_RAIL] = railTarget;
        if (moved) { builtFuzz = c.fuzz; builtGuitar = c.guitar; }
        return moved;
    }
    bool start(double T, bool warm) override {
        baseStep = T * kOversample;
        rail = railTarget;
        if (!stageG.start(baseStep, 0.0, warm)) return false;
        jfetRest();
        frontRest = rail - kDrainOhms * drainAmps;
        if (!stageB.start(T, frontRest, warm)) return false;
        return stageC.start(T, stageB.rest, warm);
    }
    void rebuild(double T) override { stageG.rebuild(T * kOversample); stageB.rebuild(T); stageC.rebuild(T); }
    void refresh() override { stageG.refresh(); stageB.refresh(); stageC.refresh(); }
    void prepare(double noise) override {
        stageG.core.prepare(noise);
        stageB.core.prepare(0.0);
        stageC.core.prepare(0.0);
    }
    // The drain's Thevenin voltage about its rest point, upsampled by the
    // caller. The rail glides over a control block like the stages' inputs.
    double front(double emf) override {
        rail += (railTarget - rail) * (1.0 / 32.0);
        double gate = stageG.core.step(emf) - stageG.rest;
        return rail - kDrainOhms * jfetStep(gate) - frontRest;
    }
    double step(double swing) override {
        double b = stageB.core.step(swing + frontRest);
        double c = stageC.core.step(b);
        return kOutputGain * (c - stageC.rest);
    }
    bool finite() const override {
        return stageG.core.finite() && stageB.core.finite() && stageC.core.finite()
            && std::isfinite(sourceVolts) && std::isfinite(drainAmps);
    }
    double noiseScale() const override { return 0.3; }
    // Q1 (the JFET), Q2 Q3 Q4 (stage B), the diode pair (stage C).
    int deviceCount() const override { return 6; }
    DeviceKind deviceKind(int index) const override {
        return index == 0 ? DEVICE_NJFET : index < 4 ? DEVICE_NPN : DEVICE_DIODE;
    }
    void sense(float forward[kMaxDevices], float reverse[kMaxDevices]) const override {
        forward[0] = (float)drainAmps; reverse[0] = 0.f;
        for (int q = 0; q < 3; ++q) { forward[1 + q] = (float)stageB.core.current[2 * q]; reverse[1 + q] = (float)stageB.core.current[2 * q + 1]; }
        for (int d = 0; d < 2; ++d) { forward[4 + d] = (float)stageC.core.current[d]; reverse[4 + d] = 0.f; }
    }
};

} // namespace fuzzy