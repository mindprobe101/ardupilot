# Firmware provenance

- Artifact: `arducopter-CubeOrangePlus.apj` (ArduCopter 4.7.0 base)
- SHA-256: `7acb2d49ea01e7c095586dddc6916abc71602fc4c769375dd7a5a2bbd51e9cee`
- Source: branch commit `0f23e9e308` (Copter-4.7.0-zhiann-bms); the
  artifact and this manifest are committed on top of it, no other
  changes
- Board: CubeOrangePlus (board id 1063)
- Toolchain: GNU Arm Embedded 10.2.1 (gcc-arm-none-eabi-10-2020-q4-major)
- Build (both steps required):

      ./waf configure --board CubeOrangePlus --extra-hwdef=zhiann-bms/extra-hwdef.dat
      ./waf copter

- Flash: 1,719,044 used / 247,028 free
- Reproducibility: built twice from clean (waf clean between) -
  byte-identical, same SHA-256
- Feature policy: Lua scripting ENABLED (operator requirement);
  ExternalAHRS backends VectorNav / MicroStrain5 / MicroStrain7 /
  InertialLabs / GSOF / SensAItion DISABLED (SBG - an upstream backend
  as of 4.7 - is the only external AHRS on this platform and stays
  enabled)
- MIGRATION: this build uses BATT_MONITOR=33 for the Zhiann driver
  (the 4.6.3 firmware used 30; see README.md)
