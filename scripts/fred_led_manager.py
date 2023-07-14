import rospy
from std_msgs.msg import Int16, Bool, Float32


from enum import Enum, IntEnum

abort_distance = False
abort_manual = 0
main_state = -5
goal_reached = False 
last_goal_reached = False

abort = True
abort_status = False
last_abort_status = False

LED_ON_TIME =  rospy.Duration(2)
start_timer = rospy.Time(0)
led_goal_reached = False 


class Fred_color(IntEnum):
            
    white = 0
    blue = 1 
    yellow  = 5
    pink = 4 
    green =  3 
    orange  = 6 
    red  = 2 
    black = -1 

def call_abort_distance(msg):
    global abort_distance 
    abort_distance = msg.data 

def call_abort_manual(msg):
    global last_abort_status
    global abort_status
    global abort_manual

    abort_status = msg.data

    if(abort_status > last_abort_status): 
        abort_manual = not abort_manual

    last_abort_status = abort_status

def call_main_state(msg):
    global main_state
    main_state = msg.data 
    print(main_state)

def call_goal_reached_callback(msg):
    global start_timer
    global last_goal_reached
    global led_goal_reached

    goal_reached = msg.data 

    if(goal_reached > last_goal_reached): 
        start_timer = rospy.Time.now()
        led_goal_reached = True

    last_goal_reached = goal_reached 

    #check if duratation of led on is over 
    if(rospy.Time.now() - start_timer >  LED_ON_TIME):
        # print("--------------------------------")

        led_goal_reached = False
    
    # print(f"{rospy.Time.now() - start_timer} { LED_ON_TIME}")




    
if __name__ == '__main__':

   #abort from ultrassonic 

   rospy.init_node('led_manager')
   rate = rospy.Rate(50)  
   while not rospy.is_shutdown():

    led_color = Fred_color.red

    pub_fita_led = rospy.Publisher("/cmd/led_strip/color",Float32,queue_size = 5)

    rospy.Subscriber('/safety/abort/distance', Bool, call_abort_distance)

    #abort from manual break 
    rospy.Subscriber('/joy/controler/ps4/break', Int16, call_abort_manual)

    #current state from main machine state 
    rospy.Subscriber('/machine_state/main', Int16, call_main_state)

    #goal reached 
    rospy.Subscriber("/goal_manager/goal/cone/reached", Bool, call_goal_reached_callback)

    
    
  
    if(main_state== 50):

        led_color = Fred_color.blue

        if(led_goal_reached):
            led_color = Fred_color.green 
        if(abort_distance):
            led_color = Fred_color.distance

        if(abort_manual):
            led_color = Fred_color.red
       


            

    if(main_state== 0):

            led_color = Fred_color.white

            if(led_goal_reached):
                led_color = Fred_color.green 

            if(abort_distance):
                led_color = Fred_color.distance

            if(abort_manual):
                 led_color = Fred_color.red



    # print(f"|{led_color}|  ABORT: DISTANCE {abort_distance}, MANUAL {abort_manual}, STATE {main_state}, GOAL_REACHED {led_goal_reached}")

    pub_fita_led.publish(led_color)

    rate.sleep()
