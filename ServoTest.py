import time
from AlphaBot import AlphaBot

if __name__ == '__main__':
    # Create an instance of AlphaBot.
    bot = AlphaBot()

    try:
        # Infinite loop to move servos.
        while True:
            print("Moving servo left...")
            bot.servo.move_left()
            time.sleep(1)
            
            print("Moving servo right...")
            bot.servo.move_right()
            time.sleep(1)
            
            print("Moving servo up...")
            bot.servo.move_up()
            time.sleep(1)
            
            print("Moving servo down...")
            bot.servo.move_down()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting loop and stopping robot.")
        bot.stop()
