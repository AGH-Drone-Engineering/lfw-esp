manipulacja akceleracją
-FSPEED=VEL_MAX, 
pwm

IMU

zatrzymanie na zakręcie

dystans hamowania

delay między wykryciem a reakcją

vacuum, turbiny


kod:
dostęp do każdego czujnika



loop:
    odczyt czujników (tablica)
    klasyfikacja wytuacji (np. wykrycie zakrętu)
    if 90 st. // - - - | | | | | 
        Stop Robot():
        -setLeftMotorSpeed(0); 
        -setRightMotorSpeed(0); 
        Execute the turn maneuver()
        -setLeftMotorSpeed(-MAX_SPEED); 
        -setRightMotorSpeed(MAX_SPEED); 
        -sleep
        -Stop Robot()
        Resume line following():
        -Control Motors()

    if intersection
    ...




