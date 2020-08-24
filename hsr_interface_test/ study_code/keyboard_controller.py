import termios, sys , tty
import hsrb_interface
import rospy

def _getch():
   fd = sys.stdin.fileno()
   old_settings = termios.tcgetattr(fd)
   try:
      tty.setraw(fd)
      ch = sys.stdin.read(1)     #This number represents the length
   finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
   return ch

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')
tts.language = tts.ENGLISH




while 1:
    getch = _getch()
    if getch == 'a':
        omni_base.go_rel(0.0, 0.01, 0.0, 100.0)
    elif getch == 'd':
        omni_base.go_rel(0.0, -0.01, 0.0, 100.0)
    elif getch == 's':
        omni_base.go_rel(-0.01, 0.0, 0.0, 100.0)
    elif getch == 'w':
        omni_base.go_rel(0.01, 0.0, 0.0, 100.0)
    elif getch == 'j':
        omni_base.go_rel(0.0, 0.0, 0.01, 100.0)
    elif getch == 'l':
        omni_base.go_rel(0.0, 0.0, -0.01, 100.0)
    elif getch == 'i':
        omni_base.go_rel(0.0, 0.0, 0.0, 100.0)
    elif getch == 'k':
        omni_base.go_rel(0.0, 0.0, 0.0, 100.0)
    elif getch == 'p':
        tts.say('Please enter the words to speak')
        words = raw_input()
        tts.say(words)
    elif getch == 'q':
        break