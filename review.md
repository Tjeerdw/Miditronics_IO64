# Code Review — Miditronics input/output module

Date: 2026-07-02
Scope: `src/main.cpp`, `src/outputs.cpp`, `src/outputs.h`, `src/config.h`, `get_git_hash.py`, `platformio.ini`
Verified against the installed library versions in `.pio/libdeps` (Adafruit MCP23017 1.3.0, FortySevenEffects MIDI 5.0.2).

## Overall impression

The code is small, readable, and does its job: a dual-personality ESP32 module that either
scans 32/64 inputs on MCP23017 expanders and sends MIDI, or receives MIDI and drives
32/64 outputs. The recent bugfix work (input-buffer seeding, NVS error handling, MIDI
channel clamping, note-127 panic) addressed real problems. The main weaknesses are:
**configuration loaded from NVS is not validated**, **the pin-mapping code contains an
integer-underflow bug that only works by accident** (and is very likely why the MCP23017
2.x library upgrade failed), and **the menu completely blocks MIDI processing**, which can
leave organ pipes stuck on. Details below, ordered by severity.

---

## 1. Bugs and faults

### 1.1 HIGH — `setOutput()` outputs 9–16 rely on integer underflow (`outputs.cpp:94`)

```cpp
else if(outputNumber <= 16)
    ext1.digitalWrite(outputNumber-17,outputValue);
```

For outputs 9–16 this computes −8…−1, which wraps to 248…255 when passed into
`digitalWrite(uint8_t pin, ...)`. It *happens* to work because library v1.3.0 maps pins as
`bitForPin = pin % 8` and `regForPin = pin < 8 ? A : B` — so 248…255 lands on port B bits
0–7, exactly the intended pins. This is a copy-paste of the ext2 branch (`25–32 →
outputNumber-17` is correct there); the ext1 branch should be `outputNumber-1` (pins 8–15).

**Consequences:** the code is correct today *only by coincidence of this library version's
internals*. Any library that validates pin numbers or maps pins differently breaks outputs
9–16 silently. **This is almost certainly why the attempt to upgrade to MCP23017 2.3.1
failed** (noted in `platformio.ini`: "tried updating to 2.3.1, but no luck yet") — the 2.x
API takes pins 0–15 and treats other values differently.

**Fix:** change to `ext1.digitalWrite(outputNumber-1, outputValue);`. Behavior on the
current library is bit-for-bit identical, so it's a safe change, and it unblocks the
library upgrade.

### 1.2 HIGH — NVS settings are loaded without validation (`main.cpp:76-83`)

`loadNVSSettings()` only guards `MidiChannel < 1`. The other three values are trusted:

- `moduletype = (moduletypes)NVS.getInt("regmodule")` — any value other than 0/1 makes
  `moduletypeNames[moduletype]` an **out-of-bounds read** (garbage on the display) and
  disables all MIDI handling (neither `Noten` nor `Register` branch matches). The
  `//TODO: test this` comment on this line suggests it was never verified.
- `startNoot` — a stored value ≥ 128 makes `noteNames[startNoot]` an **out-of-bounds
  read** (in `setup()`'s printf and in `drawMenu()`), and silently disables all note
  handling because no pitch can satisfy the range check.
- `registerOffSet` — no bounds check; large values can make the register range collide
  with the CC-value-127 "Generaal Reset".
- `MidiChannel` — the upper bound (> 16) is not checked.

**Related first-boot bug:** on a factory-fresh device (keys missing), `NVS.getInt()`
returns 0, so `loadNVSSettings()` **overwrites the in-code defaults**: `startNoot`
becomes 0 (C-1) instead of the intended 23, and `registerOffSet` becomes 0 while the menu
minimum is 1. The initializers `startNoot = 23` and `registerOffSet = 0` are effectively
dead code.

**Fix:** clamp every value after loading (channel 1–16, moduletype 0–1, startNoot within
0…127−channels, registerOffSet 1…127−channels), and fall back to sensible defaults when a
key is missing (ArduinoNvs has `getInt(key, default)` semantics via checking existence, or
simply treat 0/out-of-range as "use default").

### 1.3 MEDIUM-HIGH — the menu blocks all MIDI processing (`main.cpp:510-512`)

```cpp
while(MenuActive){ //just stay in the menu, nothing else to do
    menuCall();
}
```

The comment is not quite true. While someone browses the menu:

- An **output module stops calling `MIDI.read()`**. NoteOffs arriving during that time
  first fill the 256-byte UART RX buffer, then get dropped. If the organ is being played,
  pipes can stay on for up to 10 s (menu timeout) — or indefinitely for the dropped
  NoteOffs. That's the stuck-pipe scenario the note-127 panic was added for, built right
  into the UI.
- An input module stops scanning, so key presses/releases during menu use are delayed
  (edge detection catches up afterwards, but timing is lost).

**Fix options** (pick one):
1. Simplest: call `MIDI.read()` (and `inputModuleCall()` for input modules) inside the
   menu loop. Handlers don't touch the display except via flags, so the only interference
   is `updateForScreen`, which can be suppressed while `MenuActive`.
2. Cleaner: make the menu non-blocking — `menuCall()` is already structured so that the
   `while` could simply become part of `loop()`.

### 1.4 MEDIUM — off-by-one: last output never initialized (`outputs.cpp:76`)

```cpp
for (int i=1; i<channelNumbers; i++){ setOutput(i,0); }
```

Outputs are 1-based (`setOutput(1)`…`setOutput(64)`), so this should be
`i <= channelNumbers`. Output 32 (or 64) is never explicitly driven LOW at startup. Today
the MCP23017's power-on OLAT register is 0 so it works out, but after a soft reset
(`ESP.restart()` from the menu) the expander is *not* power-cycled: a register/pipe that
was ON keeps its latch until `pinMode` re-runs — the explicit clear exists precisely for
that case and misses the last channel.

### 1.5 MEDIUM — `extendersCount()` counts *any* I2C device, and odd counts misroute outputs (`outputs.cpp:17-32`)

The function counts every ACKing address from 1 to 126 and multiplies by 16. Two issues:

1. Any non-MCP23017 device ever added to that bus inflates the channel count.
2. Counts other than 2 or 4 are broken. Example, 3 devices → `totaalModuleKanalen = 48`,
   but `extendersInit()`/`readInputs()` only handle the `>=32` and `>=64` tiers, so ext3
   is never `begin()`-initialized. Worse: `Adafruit_MCP23017::i2caddr` defaults to 0 when
   `begin()` was never called, so `setOutput(33…48)` writes to I2C address 0x20 — **which
   is ext1**. Outputs 33–48 would alias onto outputs 1–16 and fire the wrong pipes.
   With 1 device (16 channels) nothing is initialized at all.

**Fix:** probe specifically for MCP23017s at 0x20–0x23 (`ADDRESS_EXT1..4`), require the
count to be 2 or 4, and show a clear error on the OLED otherwise.

### 1.6 MEDIUM — millis() rollover bugs in `updateScreen()` (`main.cpp:103,122`)

```cpp
if (millis() > (lastScreenUpdate + screenUpdatePeriod)) ...
```

This form breaks when `millis()` wraps after ~49.7 days — relevant for a module that is
permanently powered in an organ. Around rollover, `lastScreenUpdate + period` overflows
and the comparison misbehaves (display updates stall or fire continuously for a period).
`menuCall()` already uses the correct idiom; make these consistent:

```cpp
if (millis() - lastScreenUpdate > screenUpdatePeriod) ...
```

### 1.7 LOW-MEDIUM — git hash uses the *last* 5 characters (`get_git_hash.py:9`)

`commit_hash[-5:]` takes the tail of the SHA. All git tooling (`git log --oneline`,
GitHub) abbreviates from the *front*, so the version shown on the OLED cannot be looked
up directly — you'd need `git log --format=%H | grep 'xxxxx$'`. Use
`git rev-parse --short HEAD` instead. Also consider appending a marker (e.g. `+`) when
the working tree is dirty, so a device flashed with uncommitted changes is identifiable.

### 1.8 LOW — MIDI.begin() briefly re-routes Serial2 to the THRU pins (`main.cpp:480-484`)

The mystery in the comments ("serial voor en na midi.begin zetten lijkt betrouwbaar") has
a concrete explanation: `MIDI.begin()` internally calls `Serial2.begin(31250)` *without
pin arguments*, and on the ESP32 core UART2 then defaults to **pins 16/17 — which are the
MIDI THRU port pins on this board** (`MIDI_TH_RX_PIN`/`MIDI_TH_TX_PIN`). So:

- the **second** `Serial2.begin(31250, SERIAL_8N1, 18, 19)` is load-bearing — it moves
  the UART back to the MIDI-IN pins; keep it (with a comment saying why);
- the **first** `Serial2.begin(...)` before `MIDI.begin()` is redundant and can go.

### 1.9 LOW — error paths print to an uninitialized Serial and halt invisibly (`main.cpp:427-430, 452-456`)

- `Serial.println("SSD1306 allocation failed")` runs before any `Serial.begin()` (which
  only ever happens under `SERIALDEBUG`/`SERIALMIDI`), so the message goes nowhere. And
  since the display just failed, the `for(;;)` halt is completely silent — the module
  looks dead with no clue.
- The NVS failure halt is reasonable, but consider `nvs_flash_erase()` + retry once
  before giving up: a corrupted NVS partition is recoverable, a bricked module in an
  organ loft is annoying.

### 1.10 LOW — register reset vs. register range can collide (via NVS only)

The menu clamps `registerOffSet < 128 − totaalModuleKanalen`, so CC value 127 (the
"Generaal Reset"/"alle registers aan" trigger) is never a valid register — good design.
But an out-of-range NVS value (see 1.2) can break that invariant, making value 127 both
"reset everything" *and* "switch register N" in the same handler call. The validation fix
in 1.2 covers this; also worth documenting the "value 127 is reserved" rule in the README.

### 1.11 COSMETIC — display clear strings differ (`main.cpp:107` vs `125`)

The note/register area is cleared with 7 spaces before printing but only 6 spaces on
timeout. `"Reg 127"` is 7 characters at text size 3, so the timeout clear can leave one
character standing. Use the same 7-space string (or `fillRect`) in both places.

---

## 2. Inconsistencies and dead code

- **Dead variables** (`main.cpp:30-34`): `menuButtonState`, `menuButtonlastState`,
  `lastDebounceTime`, `debounceDelay` — leftovers from manual debouncing; ezButton
  handles this now.
- **Dead stores**: `velocity = 127; //ter ere van Hendrikus` in both note handlers
  assigns a parameter that is never used afterwards. (If the joke should stay, make it a
  comment, not code.)
- **Dead config**: `MAX_DEPTH`, `fontX`, `fontY`, `LARGE_FONT` in `config.h` are unused
  (old menu-library leftovers). `MIDI_TH_*` pins are unused in code (hardware thru) —
  fine, but a comment saying "hardware only" would help.
- **Misleading comment** (`main.cpp:473`): "Midi init, listen Omni" — `MIDI.begin(MidiChannel)`
  filters to the configured channel (which is the *correct* behavior; the comment is wrong).
  Relatedly, the handlers ignore `incomingChannel`; that's fine given the library filter,
  but worth a one-line comment because it looks like a bug.
- **Default vs. comment mismatch** (`main.cpp:44`): `startNoot = 23` while the comment
  says "24 = C1". Combined with issue 1.2 (first boot forces it to 0), the effective
  default is unclear — decide what it should be and enforce it.
- **Asymmetric menu limits**: `startNoot` can go down to 0 but `registerOffSet` stops at
  1; first-boot NVS value for both is 0. Choose the intended minimums and apply them in
  both menu and NVS validation.
- **`#define eindNoot (startNoot+totaalModuleKanalen)`** (`main.cpp:64`): a macro that
  looks like a variable. Works, but surprising; an inline function or computing it
  locally would be clearer.
- **Language mix**: identifiers and comments switch between Dutch and English
  (`inputbuffertje`, `totaalModuleKanalen`, `menuTimout` [sic], "Midi Though" [sic]).
  Purely cosmetic, but picking one language (and fixing the typos) helps future readers —
  including future AI assistants.
- **`//test` stray comment** at `main.cpp:12`.

---

## 3. Risks (operational / structural)

1. **Stuck pipes are the failure mode that matters.** Current stuck-note windows: menu
   open (1.3), MIDI cable/merger failure mid-note, input-module reboot while keys are
   held, I2C read glitch producing a phantom NoteOn (the startup seed fixed boot-time
   floods, but a runtime `readGPIOAB()` glitch after a NACK can still inject spurious
   events). Mitigations worth considering: read-twice-and-compare in `readInputs()`,
   watchdog-triggered all-off, and the panic note (already done — good).
2. **No watchdog.** If the firmware hangs (I2C lockup is the classic cause — a stuck
   slave holding SDA low will block `Wire` forever), outputs stay in their last state
   indefinitely. Enable the ESP32 task watchdog, and on boot always clear all outputs
   (which already happens — modulo bug 1.4).
3. **Frozen, EOL platform.** `espressif32 @ 3.0.0` is Arduino core 1.0.x (2021). No fixes,
   old toolchain, and growing friction with modern libraries. Not urgent, but the longer
   it waits the harder the jump. Fix 1.1 first — it is the known blocker.
4. **No tests.** The most bug-dense code (bit/pin mapping, note/register range math) is
   pure logic and trivially unit-testable, yet every change today needs hardware to
   verify. See path forward.
5. **Knowledge concentration.** The board-level facts discovered during this review
   (UART2 default-pin behavior, MCP latch behavior across soft reset, merger filtering
   CCs) live only in commit messages and comments. A short `HARDWARE.md` documenting the
   board, buses, pin table, and MIDI protocol conventions (CC 80/81 semantics, note-127
   panic, value-127 reset) would pay for itself.

---

## 4. Suggestions for improvement (code quality)

1. **Single source of truth for pin mapping.** `setOutput()` and `bitToGPIO()` are
   hand-written inverses of each other — 8 branches each, and one of them already has bug
   1.1. Derive both from one mapping function or table (channel → {chip, pin}), and unit
   test that they are inverses. This is the highest-leverage refactor in the codebase.
2. **Array of expanders** instead of `ext1..ext4` + branch ladders:
   `Adafruit_MCP23017 ext[4];` collapses `extendersInit`, `setOutput`, `readInputs` into
   short loops and makes the 32-input TODO trivial.
3. **Split `main.cpp`** (520 lines): `menu.cpp`, `midi_handlers.cpp`, `screen.cpp`,
   `settings.cpp` (NVS load/save/validate in one place). Small files, one job each.
4. **Centralize settings validation** in one `clampSettings()` used after NVS load *and*
   as a safety net before save — the menu limits then become UI convenience instead of
   the only line of defense.
5. **Bounds-check `setOutput()`**: ignore (or log) `outputNumber == 0` or
   `> totaalModuleKanalen` instead of silently writing to ext1 pin 8-0.

---

## 5. Feature ideas

- **Finish 32-input support** (the existing TODO) — falls out of suggestion 4.2 almost
  for free.
- **Diagnostics screen**: menu page showing detected expanders, I2C health, last MIDI
  message received/sent, uptime. Invaluable when debugging in the organ loft with no
  laptop.
- **Panic from the front panel**: long-press a button = all outputs off. Right now a
  stuck pipe requires a MIDI source that can send note 127, or a power cycle.
- **MIDI activity indicator** on the idle screen (small blinking marker on RX/TX) —
  answers "is anything arriving at all?" instantly.
- **Startup self-test option** (menu-triggered, not automatic — you don't want all pipes
  sounding on every boot): walk each output briefly to verify wiring.
- **OTA updates over WiFi** (ESP32 has it built in): flashing modules installed inside an
  organ case via cable is painful. Could be enabled only via a menu item for safety.
- **Settings export/import via SysEx**: back up a module's configuration, clone it to a
  replacement module.

---

## 6. Path forward (recommended order)

1. **Correctness batch** (small, low-risk, high-value):
   fix 1.1 (`outputNumber-1`), 1.4 (`<=`), 1.6 (rollover idiom), 1.11 (clear string),
   remove dead code, fix comments. Each is a one-liner; verify on hardware once.
2. **Settings validation** (1.2): one `clampSettings()` function + first-boot defaults.
3. **Menu unblocking** (1.3): keep `MIDI.read()` alive during menu use.
4. **Refactor mapping + expander array** (4.1, 4.2), extract pure logic into a file with
   no Arduino dependencies.
5. **Unit tests** on the PlatformIO `native` environment for the mapping and range logic
   — from here on, regressions are caught without hardware.
6. **Platform/library upgrade**: espressif32 6.x + MCP23017 2.x — now unblocked by step 1
   and protected by step 5. Do this on its own branch with hardware soak testing.
7. **Then features**: diagnostics screen and front-panel panic first (they help all
   later work), 32-input support, OTA last.

Steps 1–3 fit in an afternoon and remove all known field-failure modes. Steps 4–6 are the
investment that makes the codebase safe to grow; steps 7 are the payoff.
