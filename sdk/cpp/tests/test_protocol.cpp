// Frame encode / decode against frames captured from a Unitree G1 arm bus on 2026-11-25. No hardware needed.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "g1motor/g1motor.hpp"

using namespace g1motor;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAILED line %d: %s\n", __LINE__, #cond); ++failures; } } while (0)

static std::vector<std::uint8_t> hex(const std::string& s) {
    std::vector<std::uint8_t> out;
    for (std::size_t i = 0; i + 1 < s.size(); i += 2) out.push_back(static_cast<std::uint8_t>(std::strtoul(s.substr(i, 2).c_str(), nullptr, 16)));
    return out;
}
static bool same(const std::array<std::uint8_t, kCmdLen>& a, const std::string& h) {
    const auto b = hex(h);
    return b.size() == a.size() && std::equal(a.begin(), a.end(), b.begin());
}

int main() {
    // checksums
    const auto zt = hex("feee90800000000000000000000000009b1e4106");
    CHECK(crc32_core(zt.data(), 16) == 0x06411E9Bu);
    const auto rep = hex("fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c");
    CHECK(crc32_core(rep.data() + 2, 20) == 0x8CAD6325u);

    // encode matches captured commands
    MotorCmd idle; idle.id = 0;
    CHECK(same(idle.encode(), "feee000000000000000000000000000080462079"));
    MotorCmd run; run.id = 0; run.mode = kRun; run.flag = true; run.byte3 = 0x80;
    CHECK(same(run.encode(), "feee90800000000000000000000000009b1e4106"));
    MotorCmd damp = run; damp.kd = 25;
    CHECK(same(damp.encode(), "feee9080000000000000000000001900541ed614"));
    MotorCmd hold = run; hold.pos = 25032; hold.kp = 450; hold.kd = 10;
    CHECK(same(hold.encode(), "feee908000000000c8610000c2010a0071dc7c64"));
    MotorCmd restart; restart.id = 0; restart.mode = kRestart;
    CHECK(same(restart.encode(), "feee7000000000000000000000000000378838d7"));   // as sent by the robot after calibration

    // round trip
    MotorCmd c; c.id = 6; c.mode = kRun; c.flag = true; c.byte3 = 0x90; c.tau = -12; c.vel = 345; c.pos = -123456; c.kp = 300; c.kd = 80;
    const auto f = c.encode();
    const auto d = MotorCmd::decode(f.data(), f.size());
    CHECK(d && d->id == 6 && d->mode == kRun && d->flag && d->byte3 == 0x90 && d->tau == -12 && d->vel == 345 && d->pos == -123456 && d->kp == 300 && d->kd == 80);

    // decode replies
    const auto s = MotorState::decode(rep.data(), rep.size());
    CHECK(s && s->id == 2 && s->mode == 1 && s->pos == -180794 && s->vel == -8 && s->tau == -1);
    CHECK(s && s->temp_coil == 36 && s->temp_shell == 35 && s->voltage() == 48.5 && !s->fault());
    const auto fault = hex("fcee042626300000fdffc15d010008000000a80b0000a190371a");
    const auto fs = MotorState::decode(fault.data(), fault.size());
    CHECK(fs && fs->fault() && fs->error == 0x8u && fs->mode == kIdle && fs->voltage() == 24.0);

    // bad frames
    auto broken = rep; broken[10] ^= 1;
    CHECK(!MotorState::decode(broken.data(), broken.size()));
    CHECK(!MotorState::decode(rep.data(), rep.size() - 1));
    auto noisy = hex("feee000000000000000000000000000080462079fcee00");
    noisy.insert(noisy.end(), rep.begin(), rep.end());
    const auto found = find_reply(noisy.data(), noisy.size());
    CHECK(found && found->id == 2);
    CHECK(!find_reply(zt.data(), zt.size()));

    // range checks
    bool threw = false;
    try { MotorCmd bad; bad.id = 4; bad.kp = 40000; bad.encode(); } catch (const std::out_of_range&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { MotorCmd bad; bad.id = 15; bad.encode(); } catch (const std::out_of_range&) { threw = true; }
    CHECK(threw);

    // joints
    Joint j(25, -188529, 236437);
    const auto zero = j.zero;
    CHECK(Joint(16).turn_counts() == 524288 && j.turn_counts() == 819200);
    CHECK(j.sync(830810) == 1 && *j.low == -188529 + 819200 && *j.high == 236437 + 819200 && j.zero == zero + 819200);
    CHECK(j.sync(830810) == 0);
    Joint w(16, -94259, 236635);
    CHECK(w.counts(w.angle(123456)) == 123456);
    threw = false;
    try { w.sync(236635 + 100000); } catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
    Joint m(25, 0, 400000); m.margin = 40000;
    CHECK(m.clamp(-5) == 40000 && m.clamp(999999) == 360000);

    std::printf(failures ? "%d check(s) failed\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
