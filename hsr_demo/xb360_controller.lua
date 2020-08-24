#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local util = require'util'
local xbox360 = require 'xbox360'
local signal = require'signal'.signal
local gettime = require'unix'.time
local rospub = require 'rospub'

require'hcm'

local product_id = 0x0291
--local product_id = 0x028e
--local product_id = 0x0719
--local product_id = 0x02a9

xbox360.open(product_id)


local tDelay = 0.005*1E6
local running = true

-- Cleanly exit on Ctrl-C
local function shutdown()
	io.write('\n Soft shutdown!\n')
	running=false
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)


local seq=0
local head_pitch=0
local t_last_head_toggle=0
local t_last_sent=Body.get_time()
local t_last_update=Body.get_time()
local t_last = gettime()
local t_last_debug = gettime()
local t_debug_interval = 1
local t_last_button= gettime()



rospub.init('xb360_pub')
local slam_started=false
local slam_ended=false
local targetvel={0,0,0}



local function update(ret)
  local t = Body.get_time()
  local dt = t-t_last
  t_last=t

--BACK START LB RB XBOX X Y A B
  local x_db,x_max = 10,255
  local y_db,y_max = 8000,32768

  --Left/right trigger: forward and backward
  --Left stick L/R: rotate
  --Left stick U/D: look up/down
  --Right stick L/R: strafe

	local lt =( util.procFunc(ret.trigger[1],x_db,x_max )/ (x_max-x_db))
	local rt =( util.procFunc(ret.trigger[2],x_db,x_max )/ (x_max-x_db))
	local lax =util.procFunc(ret.lstick[1],y_db,y_max )/(y_max-y_db)
	local lay =util.procFunc(ret.lstick[2],y_db,y_max )/(y_max-y_db)
	local rax =util.procFunc(ret.rstick[1],y_db,y_max )/(y_max-y_db)
	local ray =util.procFunc(ret.rstick[2],y_db,y_max )/(y_max-y_db)
	targetvel={(rt-lt) * 0.40,ray*0.30,lay*0.6}

  head_pitch=head_pitch + lax * 20 * dt
  if head_pitch>30 then head_pitch=30 end
  if head_pitch<-50 then head_pitch=-50 end
  if math.abs(lax)>0 then
    hcm.set_head_target({0,head_pitch,0.3})
    hcm.set_head_execute(1)
  end

  if t-t_last_button>0.3 then
    if ret.buttons[4]>0 then
      hcm.set_logger_enable(1)
      t_last_button=t
    end

    --BACK button: start mapping process
    if ret.buttons[1]>0 then
			if slam_started==false then
	      rospub.voice("mapping started")
	      print("MAPPING START!!!!!!")
				rospub.hsrbcommand(1)
				slam_started=true

				--Now offloaded to datasend_compressed.py and datasend_uncompressed.py
	      -- os.execute("screen -S slam -X quit")
	      -- local str="screen -S slam -L -dm ./mapping_start.sh"
	      -- os.execute(str)

			else
				-- if slam_ended=false then
					rospub.voice("mapping end")
					print("MAPPING END!!!!!!")
					slam_ended=true
					slam_started=false
					rospub.hsrbcommand(2)
					-- local str="screen -S slamsave -L -dm ./mapping_end.sh"
					-- os.execute(str)
					-- unix.usleep(1E6*1)
					-- os.execute("screen -S slam -X quit")

				-- end
			end
      t_last_button=t + 1.0 --don't allow re-press for 1 second
    end

    --Left shoulder button: add current position to the pose list
    if ret.buttons[3]>0 then
      rospub.voice("Pose")
      rospub.poselog(1)
      print("Pose recorded")
      t_last_button=t+0.7 --1 sec delay
    end

    if ret.buttons[4]>0 then --Right shoulder button: Image grab
      rospub.imagelog(1)
      t_last_button=t+0.2 --0.5 sec delay
    end


    if ret.buttons[6]>0 then --X button: Gripper close
      local cur_gripper_target=hcm.get_gripper_target(2)
      if cur_gripper_target>0 then
        hcm.set_gripper_target(0)
      else
        hcm.set_gripper_target(1)
      end
      hcm.set_gripper_execute(1)
      t_last_button=t
    end

    if ret.buttons[7]>0 then --Y button: Proceed command
      -- if wcm.get_object_num()>0 and wcm.get_object_selected_no()>0 then
      --   wcm.set_target_test(0)
      --   body_ch:send'pickup'
      --   t_last_button=t
      -- else
			--
			--
      -- end
			-- wcm.set_game_task1_stage(0) --start challenge
			-- game_ch:send'task3'
    end

		if ret.buttons[8]>0 then --A button: Scan for objects
			-- body_ch:send'scanobjects'


			wcm.set_game_proceed(1)

      t_last_button=t
		end

    if ret.buttons[9]>0 then --B button: Scan for door handle
      body_ch:send'scanfridge'
      t_last_button=t
    end

    -- if ret.dpad[1]>0 then
    --   -- rospub.voice("Arm forward position")
    --   -- print("Arm forward position")
    --   -- rospub.armcontrol({0.05,0,-0*DEG_TO_RAD,-90*DEG_TO_RAD,0},2.0)
    --   body_ch:send'handover'
    --   t_last_button=t
    -- elseif ret.dpad[1]<0 then
    --   if ret.buttons[6]>0 then --X button: Gripper close
    --   rospub.voice("State Reset")
    --   body_ch:send'reset'
    --   t_last_button=t
    if ret.buttons[5]>0 then --XBOX button: state reset
      rospub.voice("State Reset")
      body_ch:send'reset'
      t_last_button=t

    elseif ret.dpad[2]<0 then
      hcm.set_head_cycletarget(1)
      hcm.set_head_looktarget(1)
      t_last_button=t
    elseif ret.dpad[2]>0 then
      hcm.set_head_cycletarget(-1)
      hcm.set_head_looktarget(1)
      t_last_button=t
    end

  end

	if t-t_last_debug>t_debug_interval then
    print(
      string.format(
      "Dpad:%d %d LStick:%.1f %.1f RStick:%.1f %.1f LT%.1f RT%.1f ",
      ret.dpad[1],ret.dpad[2],
      lax,lay,rax,ray,lt,rt)
      ..
      string.format("Buttons:%d%d%d%d%d%d%d%d%d",
        unpack(ret.buttons))
    )
    t_last_debug=t
  end
	seq=seq+1






end


local last_control_t = Body.get_time()

local function sendcmd()
  local t=Body.get_time()
  local dt=t-t_last_sent
  local vel=hcm.get_base_velocity()

  vel[1]=util.procFunc(targetvel[1]-vel[1],0,dt*0.20)+vel[1]
  vel[2]=util.procFunc(targetvel[2]-vel[2],0,dt*0.20)+vel[2]
  vel[3]=util.procFunc(targetvel[3]-vel[3],0,dt*0.45)+vel[3]
  vel[1]=util.procFunc(vel[1],0,0.45)
  vel[2]=util.procFunc(vel[2],0,0.45)
  vel[3]=util.procFunc(vel[3],0,0.60)
  local d1=math.sqrt(vel[1]*vel[1]+vel[2]*vel[2])
  local d2=math.sqrt(targetvel[1]*targetvel[1]+targetvel[2]*targetvel[2])

  if not (targetvel[1]==0 and targetvel[2]==0 and targetvel[3]==0) then
    last_control_t = t
    -- print"UPD"
  end

  if t-last_control_t<1.5 then
    hcm.set_base_teleop_t(t)
    hcm.set_base_velocity({vel[1],vel[2],vel[3]})
    rospub.baseteleop(vel[1],vel[2],vel[3])
  end


  if hcm.get_head_execute()==1 then
    local headTarget=hcm.get_head_target()
    rospub.headcontrol(headTarget[1]*math.pi/180,headTarget[2]*math.pi/180,headTarget[3])
    hcm.set_head_execute(0)
  end
  t_last_sent=t
end

while running do
  local ret = xbox360.read()
  update(ret)
  sendcmd()
  unix.usleep(1e6*0.05) --20fps
end

-- if gcm.get_game_selfdestruct()==1 then
-- 	-- os.execute("mpg321 ~/Desktop/NaverRobotics/media/selfdestruct.mp3")
-- 	-- os.execute('sync')
-- 	-- os.execute('systemctl poweroff -i')
-- end
