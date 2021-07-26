Ardupilot optimAero External library
-to be used to read arduinos
-to be used to read external imus
-to be used to read external computers

**would like for this to somehow interface with AHRS/BatteryMonitor
**would like for this to interface directly with logging as well!

**this was done as opposed to other attempts to keep code in Ardu branch to a minimal - pain in ass everytime you want to update. If we keep it as library hopefully it will be easier to maintain in long run.


--Places where we will have to inject code
1) SerialManager.cpp/.h -> need to add our own serial type so MP can initialize it
2) Copter.h -> add OA_STate, OA includes, some possible functions to be init and run as well
3) Copter.cpp -> add to scheduler -> or add to usercode....

4) APM_CONFIG.h -> update user fncs
5) Parameters.cpp/h -> oa module add
6) UserCode.cpp/h -> add oa module calls




Serial mode == 40
battery mode = 21