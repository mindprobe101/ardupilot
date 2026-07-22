# Firmware provenance

- Artifact: `arducopter-CubeOrangePlus.apj`
- SHA-256: `07e4f0a4fa9bea1736bba4aa0e9821fbbecd6f1134725a70f6d3ce20b80992c8`
- Source: branch commit `66d9ccbb04` (AP_BattMonitor: ZhiannBMS: coherence
  and current-fault refinements), no uncommitted changes
- Board: CubeOrangePlus (board id 1063)
- Toolchain: GNU Arm Embedded 10.2.1 (gcc-arm-none-eabi-10-2020-q4-major)
- Build (both steps required; the extra hwdef disables unused Lua
  scripting - without it the image does not fit):

      ./waf configure --board CubeOrangePlus --extra-hwdef=zhiann-bms/extra-hwdef.dat
      ./waf copter

- Flash: 1,785,232 used / 180,816 free
- Reproducibility: built twice from clean (waf clean between) -
  byte-identical, same SHA-256
- Feature note: AP_SCRIPTING_ENABLED=0 - SCR_* parameters absent by design
