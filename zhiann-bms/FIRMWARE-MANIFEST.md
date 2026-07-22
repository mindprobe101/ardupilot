# Firmware provenance

- Artifact: `arducopter-CubeOrangePlus.apj`
- SHA-256: `378441c200936f1d6866d4685399dc5dcd3ece6b2ab9b2d561381988309d5788`
- Source: branch commit `510378371e` plus the extra-hwdef change committed
  with this manifest; no other changes
- Board: CubeOrangePlus (board id 1063)
- Toolchain: GNU Arm Embedded 10.2.1 (gcc-arm-none-eabi-10-2020-q4-major)
- Build (both steps required):

      ./waf configure --board CubeOrangePlus --extra-hwdef=zhiann-bms/extra-hwdef.dat
      ./waf copter

- Flash: 1,944,912 used / 21,168 free
- Reproducibility: built twice from clean (waf clean between) -
  byte-identical, same SHA-256
- Feature policy: Lua scripting ENABLED (operator requirement);
  ExternalAHRS backends VectorNav / MicroStrain5 / MicroStrain7 /
  InertialLabs DISABLED (SBG is the only external AHRS on this
  platform; frees ~22 KB, image does not fit with everything on)
