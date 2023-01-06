import cv2
import math
import numpy as np
import time
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib

pos_azul=[0,0]
pos_amarillo=[0,0]
veces = int(0)
veces1 = int(0)

GPIO.setmode(GPIO.BCM) 
GPIO.setwarnings(False)

pulsador = 8
pwm_gpio = 17
pwm_gpio_2 = 18
pwm_gpio_3 = 27
frequence = 50
GPIO.setup(pulsador, GPIO.IN)
GPIO.setup(pwm_gpio, GPIO.OUT)
GPIO.setup(pwm_gpio_2, GPIO.OUT)
GPIO.setup(pwm_gpio_3, GPIO.OUT)
pwm = GPIO.PWM(pwm_gpio, frequence)
pwm_2 = GPIO.PWM(pwm_gpio_2, frequence)
pwm_3 = GPIO.PWM(pwm_gpio_3, frequence)

direction= 22 # Direction (DIR) GPIO Pin
step = 23 # Step GPIO Pin
EN_pin = 24 # enable pin (LOW to enable)
GPIO.setup(EN_pin,GPIO.OUT)
GPIO.output(EN_pin,GPIO.LOW)



while True:
    
    if GPIO.input(pulsador) == False:
            
        for opcion in range (3):
            
            if opcion ==0:
                cap = cv2.VideoCapture(0)
                azulBajo = np.array([100,100,20],np.uint8)
                azulAlto = np.array([125,255,255],np.uint8)
                while True:
                    ret,frame = cap.read()
                    if ret==True:
                        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                        mask = cv2.inRange(frameHSV,azulBajo,azulAlto)
                        contornos, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if veces <= 10:
                            for c in contornos:
                                area = cv2.contourArea(c)
                                if area > 220:
                                    M = cv2.moments(c)
                                    if (M["m00"]==0): M["m00"]=1
                                    x1 = int(M["m10"]/M["m00"])
                                    y1 = int(M['m01']/M['m00'])
                                    print('las coordenadas del cubo azul son: (',x1,',',y1,')')
                                    pos_azul[0]=x1
                                    pos_azul[1]=y1
                                    cv2.circle(frame, (x1,y1), 7, (255,0,0), -1)
                                    font = cv2.FONT_HERSHEY_SIMPLEX
                                    cv2.putText(frame, '{},{}'.format(x1,y1),(x1+10,y1), font, 0.75,(255,0,0),1,cv2.LINE_AA)
                                    nuevoContorno = cv2.convexHull(c)
                                    cv2.drawContours(frame, [nuevoContorno], 0, (255,0,0), 3)
                                    veces = veces + 1
                                    print(veces)
                                cv2.imshow('frame',frame)
                                if cv2.waitKey(1) & 0xFF == ord('s') :
                                    break
                        print('veces: ',veces)       
                        if veces == 10:
                            ret = False
                    if veces == 10:
                        print('Cerrando')
                        cv2.destroyAllWindows()
                        break  
                cap.release()
                time.sleep(5)
                          
            
            if opcion == 1:
                cap = cv2.VideoCapture(0)
                amarilloBajo = np.array([15,100,20],np.uint8)
                amarilloAlto = np.array([45,255,255],np.uint8)
                while True:
                    ret,frame = cap.read()
                    if ret==True:
                        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                        mask = cv2.inRange(frameHSV,amarilloBajo,amarilloAlto)
                        contornos, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if veces1 <= 10:
                            for c in contornos:
                                area = cv2.contourArea(c)
                                if area > 400:
                                    M = cv2.moments(c)
                                    if (M["m00"]==0): M["m00"]=1
                                    x2 = int(M["m10"]/M["m00"])
                                    y2 = int(M['m01']/M['m00'])
                                    print('las coordenadas del cubo amarillo son: (',x2,',',y2,')')
                                    pos_amarillo[0]=x2
                                    pos_amarillo[1]=y2
                                    cv2.circle(frame, (x2,y2), 7, (0,255,200), -1)
                                    font = cv2.FONT_HERSHEY_SIMPLEX
                                    cv2.putText(frame, '{},{}'.format(x2,y2),(x2+10,y2), font, 0.75,(0,255,200),1,cv2.LINE_AA)
                                    nuevoContorno = cv2.convexHull(c)
                                    cv2.drawContours(frame, [nuevoContorno], 0, (0,255,200), 3)
                                    veces1 = veces1 + 1
                                    print(veces1)
                                cv2.imshow('frame',frame)
                                if cv2.waitKey(1) & 0xFF == ord('s') :
                                    break
                        print('veces_uno: ',veces1)       
                        if veces1 == 10:
                            ret = False
                    if veces1 == 10:
                        print('Cerrando')
                        cv2.destroyAllWindows()
                        break  
                cap.release()
                time.sleep(5)               

        print('Variable azul',pos_azul)
        print('Variable amarillo',pos_amarillo)
        print(opcion)
        
        print('posicion x dado azul:', x1)
        print('posicion y dado azul:', y1)
        
        print('posicion x dado amarillo:', x2)
        print('posicion y dado amarillo:', y2)

        

        # PRIMERA POSICION PARA DADO AZUL
        if 462 <= x1 <= 492 and 389 <= y1 <= 419:

            # valor de px en cm
            px = 7
            # valor de py en cm
            py = 2.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
                
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 693, .0005, False, .05)
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(45))
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(77))
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 693, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
            
        # SEGUNDA POSICION PARA DADO AZUL    
        elif 480 <= x1 <= 310 and 310 <= y1 <= 340:

            # valor de px en cm
            px = 12
            # valor de py en cm
            py = 2.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 729, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(55)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(70)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 729, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)

         
        # TERCERA POSICION PARA DADO AZUL    
        elif 476 <= x1 <= 506  and 238 <= y1 <= 268:

            # valor de px en cm
            px = 17
            # valor de py en cm
            py = 2.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)


            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 738, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(80)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(67)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 738, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        # CUARTA POSICION PARA DADO AZUL    
        elif 469 <= x1 <= 499  and 161 <= y1 <= 191:

            # valor de px en cm
            px = 22
            # valor de py en cm
            py = 2.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 738, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(95)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(70)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 738, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
            
        # QUINTA POSICION PARA DADO AZUL    
        elif 389 <= x1 <= 419  and 400 <= y1 <= 430:

            # valor de px en cm
            px = 7
            # valor de py en cm
            py = 7.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 382, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(50)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(75)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 382, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            

        # SEXTA POSICION PARA DADO AZUL    
        elif 387 <= x1 <= 417  and 306  <= y1 <= 336:

            # valor de px en cm
            px = 12
            # valor de py en cm
            py = 7.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 516, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(60)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(68)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 516, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # SEPTIMA POSICION PARA DADO AZUL    
        elif 388 <= x1 <= 418  and 232 <= y1 <= 262 :

            # valor de px en cm
            px = 17
            # valor de py en cm
            py = 7.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 578, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(78)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(66)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 578, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        # OCTAVA POSICION PARA DADO AZUL    
        elif 309 <= x1 <= 429  and 163 <= y1 <= 193 :

            # valor de px en cm
            px = 22
            # valor de py en cm
            py = 7.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 631, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(100)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(70)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 631, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            

        # NOVENA POSICION PARA DADO AZUL    
        elif  302 <= x1 <= 332  and 385 <= y1 <= 415 :

            # valor de px en cm
            px = 7
            # valor de py en cm
            py = 12.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 258, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(60)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(68)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 258, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
           
        # DECIMA POSICION PARA DADO AZUL    
        elif 313 <= x1 <= 343  and 306 <= y1 <= 336 :

            # valor de px en cm
            px = 12
            # valor de py en cm
            py = 12.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 391, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(72)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(67)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 391, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)

        # DECIMA PRIMERA POSICION PARA DADO AZUL    
        elif 322 <= x1 <= 352  and 227 <= y1 <= 257:

            # valor de px en cm
            px = 17
            # valor de py en cm
            py = 12.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 480, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(95)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(68)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 480, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # DECIMA SEGUNDA POSICION PARA DADO AZUL    
        elif 329 <= x1 <= 359  and 170 <= y1 <= 200 :

            # valor de px en cm
            px = 22
            # valor de py en cm
            py = 12.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 516, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(110)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(73)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 516, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            

        # DECIMA TERCERA PARA DADO AZUL    
        elif 226 <= x1 <= 256  and 381 <= y1 <= 411:

            # valor de px en cm
            px = 7
            # valor de py en cm
            py = 17.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 196, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(80)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(66)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 196, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # DECIMA CUARTA PARA DADO AZUL    
        elif 234 <= x1 <= 264  and 303 <= y1 <= 333 :

            # valor de px en cm
            px = 12
            # valor de py en cm
            py = 17.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 302, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(90)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(67)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 302, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # DECIMA QUINTA PARA DADO AZUL    
        elif 240 <= x1 <= 270  and 222 <= y1 <= 252 :

            # valor de px en cm
            px = 17
            # valor de py en cm
            py = 17.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 391, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(110)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(70)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 391, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            

        # DECIMA SEXTA POSICION PARA DADO AZUL    
        elif 248 <= x1 <= 278  and 169 <= y1 <= 199 :

            # valor de px en cm
            px = 22
            # valor de py en cm
            py = 17.5
            # valor de pz en cm
            pz = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r = math.sqrt(px**2+py**2)
            print('el radio es: ',r)

            #ANULO 3
            d=pz-l1 #cateto 1
            #r es el cateto 2
            h1= math.sqrt(r**2+d**2) #hipotenusa
            print('la hipotenusa es: ', h1)

            #triangulo oblicuo
            cos3=(h1**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3)
            angulo3=math.atan((math.sqrt(1-math.cos(cos3)**2))/cos3)*(180.0/math.pi)
            angulo_3 = round(angulo3)
            print('angulo 3 es: ', angulo_3)

            #ANGULO 2
            angulo2=math.atan((d)/r)-math.atan((l3*math.sin(angulo3))/(l2+l3*math.cos(angulo3)))*(180.0/math.pi)
            angulo_2 = round(angulo2) 
            print('angulo 2 es: ', angulo_2)

            #ANGULO 1
            angulo1=math.atan(py/px)*(180.0/math.pi)
            angulo_1 = round(angulo1)
            print('angulo 1 es: ', angulo_1)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 436, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(130)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(80)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 436, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # PRIMERA POSICION PARA DADO AMARILLO
        if 462 <= x2 <= 492 and 389 <= y2 <= 419:

            # valor de px en cm
            px_2 = 7
            # valor de py en cm
            py_2 = 2.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 693, .0005, False, .05)
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(45))
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(77))
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 693, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
            
        # SEGUNDA POSICION PARA DADO AMARILLO    
        elif 480 <= x2 <= 495 and 310 <= y2 <= 340:

             # valor de px en cm
            px_2 = 12
            # valor de py en cm
            py_2 = 2.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 729, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(55)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(70)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 729, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)

         
        # TERCERA POSICION PARA DADO AMARILLO   
        elif 476 <= x2 <= 506  and 238 <= y2 <= 268:

             # valor de px en cm
            px_2 = 17
            # valor de py en cm
            py_2 = 2.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)



            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 738, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(80)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(67)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 738, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        # CUARTA POSICION PARA DADO AMARILLO   
        elif 469 <= x2 <= 499  and 161 <= y2 <= 191:

             # valor de px en cm
            px_2 = 22
            # valor de py en cm
            py_2 = 2.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 738, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(95)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(70)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 738, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
            
        # QUINTA POSICION PARA DADO AMARILLO   
        elif 389 <= x2 <= 419  and 400 <= y2 <= 430:

             # valor de px en cm
            px_2 = 7
            # valor de py en cm
            py_2 = 7.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 382, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(50)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(75)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 382, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            

        # SEXTA POSICION PARA DADO AMARILLO   
        elif 387 <= x2 <= 417  and 305  <= y2 <= 345:

             # valor de px en cm
            px_2 = 12
            # valor de py en cm
            py_2 = 7.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 516, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(60)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(68)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 516, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # SEPTIMA POSICION PARA DADO AMARILLO   
        elif 388 <= x2 <= 418  and 232 <= y2 <= 262 :

             # valor de px en cm
            px_2 = 17
            # valor de py en cm
            py_2 = 7.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 578, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(78)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(66)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 578, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        # OCTAVA POSICION PARA DADO AMARILLO  
        elif 389 <= x2 <= 429  and 163 <= y2 <= 193 :

             # valor de px en cm
            px_2 = 22
            # valor de py en cm
            py_2 = 7.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 631, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(100)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(70)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 631, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            

        # NOVENA POSICION PARA DADO AMARILLO   
        elif  302 <= x2 <= 332  and 385 <= y2 <= 415 :

             # valor de px en cm
            px_2 = 7
            # valor de py en cm
            py_2 = 12.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 258, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(60)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(68)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 258, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
           
        # DECIMA POSICION PARA DADO AMARILLO    
        elif 313 <= x2 <= 343  and 306 <= y2 <= 336 :

             # valor de px en cm
            px_2 = 12
            # valor de py en cm
            py_2 = 12.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 391, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(72)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(67)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 391, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)

        # DECIMA PRIMERA POSICION PARA DADO AMARILLO   
        elif 322 <= x2 <= 352  and 227 <= y2 <= 257:

             # valor de px en cm
            px_2 = 17
            # valor de py en cm
            py_2 = 12.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 480, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(95)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(68)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 480, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # DECIMA SEGUNDA POSICION PARA DADO AMARILLO   
        elif 329 <= x2 <= 359  and 170 <= y2 <= 200 :

             # valor de px en cm
            px_2 = 22
            # valor de py en cm
            py_2 = 12.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 516, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(110)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(73)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 516, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            

        # DECIMA TERCERA PARA DADO AMARILLO  
        elif 226 <= x2 <= 256  and 381 <= y2 <= 411:

             # valor de px en cm
            px_2 = 7
            # valor de py en cm
            py_2 = 17.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 196, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(80)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(66)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 196, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # DECIMA CUARTA PARA DADO AMARILLO    
        elif 234 <= x2 <= 264  and 303 <= y2 <= 333 :

             # valor de px en cm
            px_2 = 12
            # valor de py en cm
            py_2 = 17.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 302, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(90)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(67)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 302, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
        
        # DECIMA QUINTA PARA DADO AMARILLO   
        elif 240 <= x2 <= 270  and 222 <= y2 <= 252 :

             # valor de px en cm
            px_2 = 17
            # valor de py en cm
            py_2 = 17.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 391, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(110)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(70)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 391, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            

        # DECIMA SEXTA POSICION PARA DADO AMARILLO    
        elif 248 <= x2 <= 278  and 169 <= y2 <= 199 :

             # valor de px en cm
            px_2 = 22
            # valor de py en cm
            py_2 = 17.5
            # valor de pz en cm
            pz_2 = 3

            l1=24 #valor de l1
            l2=15.6 #valor de l2
            l3=18 #valor de l3

            r_2 = math.sqrt(px_2**2+py_2**2)
            print('el radio 2 es: ',r_2)

            #ANULO 3
            d_2 =pz_2-l1 #cateto 1
            #r es el cateto 2
            h1_2= math.sqrt(r_2**2+d_2**2) #hipotenusa
            print('la hipotenusa 2 es: ', h1_2)

            #triangulo oblicuo
            cos3_2=(h1_2**2-l2**2-l3**2)/(2*l2*l3)
            print(cos3_2)
            angulo3_2 = math.atan((math.sqrt(1-math.cos(cos3_2)**2))/cos3_2)*(180.0/math.pi)
            angulo_3_2 = round(angulo3_2)
            print('angulo 3 del segundo es: ', angulo_3_2)

            #ANGULO 2
            angulo2_2 =math.atan((d_2)/r_2)-math.atan((l3*math.sin(angulo3_2))/(l2+l3*math.cos(angulo3_2)))*(180.0/math.pi)
            angulo_2_2 = round(angulo2_2) 
            print('angulo 2 del segundo es: ', angulo_2_2)

            #ANGULO 1
            angulo1_2 =math.atan(py_2/px_2)*(180.0/math.pi)
            angulo_1_2 = round(angulo1_2)
            print('angulo 1 del segundo es: ', angulo_1_2)
            
            def angle_to_percent (angle) :
                if angle > 190 or angle < 0 :
                    return False
                
                start = 2.5
                end = 12.5
                ratio = (end - start)/180 

                angle_as_percent = angle * ratio

                return start + angle_as_percent
            
            
           # Inicio de los movimientos
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(False, "1/16" , 436, .0005, False, .05) ######
            
            #Posición Inicial 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(1)
            
            #Posición Inicial 0° garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(1)
            
            #Posición ingresada servo 2
            pwm_2.start(angle_to_percent(130)) ########
            time.sleep(2)

            #Posición ingresada servo 1
            pwm.start(angle_to_percent(80)) ########
            time.sleep(2)
            
            #activa garra
            pwm_3.start(angle_to_percent(0))
            time.sleep(1)
            
            #Regresa a la posición 0° servo 1
            pwm.start(angle_to_percent(0))
            time.sleep(2)
            
            mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
            mymotortest.motor_go(True, "1/16" , 436, .0005, False, .05)
            
            #Regresa a la posición 0° servo 2
            pwm_2.start(angle_to_percent(0))
            time.sleep(2)
            
            #cierra garra
            pwm_3.start(angle_to_percent(90))
            time.sleep(3)
            
    else:
        print('Presione el boton para iniciar')
        time.sleep(3)

        
        
      
            
                  
               
            
        
        
            
         

            
       
            
          
          
         
            
        
       
            
         
            
        
        
            
      
       
        
         
          
         
            
            
            
            
            
            




