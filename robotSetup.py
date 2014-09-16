try:
    import wpilib
except:
    import pyfrc.wpilib as wpilib
import threading

class potRate():
    
    def __init__(self, potObj, timerObj):
        self.rate = 0.0
        self.stopThread = False
        self.lastRate = 0.0
        self.lastPotVal = potObj.GetVoltage()
        self.potObj = potObj
        self.timerObj = timerObj
        self.timerObj.Start()
        self.timerObj.Reset()

    def calcValue(self): 
        while not self.stopThread:  
            try:
                #This tells us how much it has traveled since it was last looked at by comparing last to current
                Rate = 0.0
                lastRate = self.rate
                potVal = self.potObj.GetVoltage()
                potDiff = (self.potObj.GetVoltage() - self.lastPotVal)
                
                #The pots give some jumpy data when not moving so this is a very basic filter that seprates out any garbage in readings
                if (abs(potDiff) >= 0.02):
                    #here we actually calulate out the rate, rate is just distance travled over time, we time how fast the pot moved
                    #Becuase of the way the shooter works, we don't want negitive values
                    if (potDiff > 0):
                        Rate = potDiff/self.timerObj.Get()
                        Rate = (Rate + lastRate)/2
                    else:
                        pass
            except ZeroDivisionError:
                Rate = 0.0
                print("RPM error zero divison")
            except:
                print("RPM error unhaneled")
                self.stopThread = True
            self.rate = Rate       
            self.lastPotVal = potVal     
            self.timerObj.Reset()
            wpilib.Wait(0.01)

    def Start(self):
        self.stopThread = False
        self.rateThread = threading.Thread(target=self.calcValue, args = ())
        self.rateThread.daemon = True
        self.rateThread.start()
    def Stop(self):
        self.stopThread = True
        self.rateThread.join()
    def getRate(self):
        return self.rate

class robotFunctions():
    def gotoPosition(self, currentPosition, wantedPosition, gain, stopWindow = 0, minSpeedFoward = None, minSpeedBackward = None): 
        error = currentPosition - wantedPosition
   
        if ((error > stopWindow*-1) and (error < stopWindow)):
            return 0.0

        errorGain = error/gain
        if not ((minSpeedFoward is None) and (minSpeedBackward is None)):
            if (errorGain < windowAccuarcy*-1):
                if errorGain > minSpeedBackward:
                    return minSpeedBackward
            if (errorGain > windowAccuarcy):
                if (errorGain < minSpeedFoward):
                    return minSpeedFoward
        return errorGain 
    
    def smartDeadZone(self, jostickValue, deadZoneWindow, setMotor = 0.0):
        if (jostickValue <= deadZoneWindow*-1):
            return -1*((1/(1-deadZoneWindow))*(abs(jostickValue)-deadZoneWindow))
        if (jostickValue >= deadZoneWindow):
            return ((1/(1-deadZoneWindow))*(abs(jostickValue)-deadZoneWindow))
        return setMotor
    
    def arcadeDrive(self, joystickYAxis, joystickXAxis, deadZoneWindow = 0.0):
        joystickYAxis = self.smartDeadZone(joystickYAxis, deadZoneWindow)
        joystickXAxis = self.smartDeadZone(joystickXAxis, deadZoneWindow)
        
        rightDrive = joystickYAxis - joystickXAxis
        leftDrive = joystickYAxis + joystickXAxis
        return (rightDrive, leftDrive)
    
    def setRate(self, WantedRate, currentRate, maxRate, setMotor, rateGain,window = 0.0):
        errorRate = abs(WantedRate * maxRate) - currentRate
        errorRateGain = errorRate/rateGain
    
        if (errorRateGain < window) and (errorRateGain > (-1 * window)):
            return 0.0
        if WantedRate == 0:
            return 0.0
        
        setMotor = errorRateGain + abs(setMotor)
        return setMotor
    
    def mortorSafty(self, setMotors = []):
        newSetMotor = []
        setMotor = 0.0
        for i in range(0, len(setMotors)):
            setMotor = setMotors[i]
            if setMotor > 1:
                setMotor = 1
            if setMotor < -1:
                setMotor = -1
            newSetMotor.append(setMotor)
        return newSetMotor
    
    def lastValueJoy(self, joystickObj, numberOfButtons):
        exist = True
        lastValues=[]
        i = 1
        
        while ((i <= numberOfButtons) and exist):
            try:
                lastValues.append(joystickObj.GetRawButton(i))
            except:
                exist = False
            i += 1
            
        return lastValues
    
    def reloadCode(self):
        raise RuntimeError("reboot")
    
class variableInitalize():

    #Joysticks
    stick = wpilib.Joystick(1)
    
    #Visctors
    DriveL1 = wpilib.Victor(2)
    DriveL2 = wpilib.Victor(4)
    DriveR1 = wpilib.Victor(1)
    DriveR2 = wpilib.Victor(3)

    shooterRoller = wpilib.Victor(9)
    shooterNeck = wpilib.Victor(6)

    bridgeWedge = wpilib.Victor(10)
    elevator = wpilib.Victor(5)
    frontFeed = wpilib.Victor(7)
    backFeed = wpilib.Victor(8)

    #Encoders
    neckEncode = wpilib.Encoder(5, 6, True)
    leftDriveEncode = wpilib.Encoder(3,4,False)
    rightDriveEncode = wpilib.Encoder(1,2,False)
    turretroller = wpilib.Encoder(7,8,False)

    #Relays
    camLights = wpilib.Relay(1)
    targetInd = wpilib.Relay(4)
    compLights = wpilib.Relay(2)

    #Timers
    neckTimer = wpilib.Timer()
    ballTimer = wpilib.Timer()
    feedTimer = wpilib.Timer()
    neckcTimer = wpilib.Timer()
    rpmTime = wpilib.Timer()
    rpmlockedTimer = wpilib.Timer()
    autonTimer = wpilib.Timer()
    rpmLockFlash = wpilib.Timer()
    rpmTimer = wpilib.Timer()

    #Digital Input
    frontFeedDI = wpilib.DigitalInput(9)
    backFeedDI = wpilib.DigitalInput(10)
    bottomNeckDI = wpilib.DigitalInput(11)
    topNeckDI = wpilib.DigitalInput(12)

    #Digital Output
    #trackingLight = wpilib.DigitalOutput(12)
    ledNotification = wpilib.DigitalOutput(13)

    #Analong
    rpmPot = wpilib.AnalogChannel(1)
    bridgePot = wpilib.AnalogChannel(2)

    #Solinoid
    breakRelay = wpilib.Solenoid(1)

    #drive station
    ds = wpilib.DriverStationLCD.GetInstance()
                
    rateOfPot = potRate(rpmPot,rpmTimer)

    def Wait(self, waitTime):
        wpilib.Wait(waitTime)
    def SimpleRobot(self):
        return wpilib.SimpleRobot

class globalVar():

    #Shooter Rate 
    maxRate = 230.0
    errorRateGain = 6000.0
                
    #Dead Zones and Windows
    shooterNeckLimit = 530
    driveDeadZone = 0.05
    neckDeadZone = 0.1
    shooterDeadZone = 0.05

    #Bridge manilpulator Vars
    bridgeMin = 0.80 #This is the lowest value the bridge arm can go to this is 'Down position'
    bridgeMax = 3.80 #This is the highest value it can go to, this is 'Home position'

    bridgePositionGain = 0.6 #This is how fast it approaches any set position
    bridgeStopWindow = 0.05 #this is the lowest value that is given during positioning before it stops moveing

    bridgeMinFowardSpeed = 0.1 #this is the minimum voltage needed to have the arm move foward
    bridgeMinBackwardSpeed = 0.12 #this is the minimum voltage needed to hve the arm move backwards
                
