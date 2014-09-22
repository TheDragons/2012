import sys
import robotSetup 

robotVar = robotSetup.variableInitalize()
globVar = robotSetup.globalVar()
functions = robotSetup.robotFunctions()
#robotVar.rateOfPot.Start()
#---this is a change

class MyRobot(robotVar.SimpleRobot()):
        def Disabled(self):
                while self.IsDisabled():
                    if robotVar.stick.GetRawButton(3):
                        sys.exit()
                    
                    robotVar.Wait(0.04)

        def Autonomous(self):
                self.GetWatchdog().SetEnabled(False)
                robotVar.ds.Clear()
                
                maxRateVal = 0.0
                while self.IsAutonomous() and self.IsEnabled():

                        if robotVar.autonTimer.Get() > maxRateVal:
                            maxRateVal = robotVar.autonTimer.Get()
                        if robotVar.stick.GetRawButton(2):
                            maxRateVal = 0.0
                        robotVar.autonTimer.Reset()
                #----------------Drive Station
                        robotVar.ds.Clear()
                        robotVar.ds.Print(robotVar.ds.Line(0), 1, "AutonMode Disabled")
                        robotVar.ds.Print(robotVar.ds.Line(1), 1, "max loop:" + str(maxRateVal))
                        robotVar.ds.UpdateLCD()
                #----------------Zero Me out

                        robotVar.Wait(0.01)

        def OperatorControl(self):
                dog = self.GetWatchdog()
                dog.SetEnabled(False)
                
                robotVar.ds.Clear()
                
            #----------Encoder Reseting and inializing
                robotVar.neckEncode.Start()
                robotVar.neckEncode.Reset()
                robotVar.leftDriveEncode.Start()
                robotVar.leftDriveEncode.Reset()
                robotVar.rightDriveEncode.Start()
                robotVar.rightDriveEncode.Reset()
                robotVar.turretroller.Start()
                robotVar.turretroller.Reset()
                
            #-------Timer Reseting and Initalizing
                robotVar.rpmTime.Stop()
                robotVar.rpmTime.Reset()
                robotVar.rpmTime.Start()
                robotVar.autonTimer.Reset()
                robotVar.autonTimer.Start()

            #--------Shooter's Rate Conrol
                wantedRate = 0.0
                lastRate = 0.0
                potDiff = 0.0
                maxRateVal = 0.0
                lastRpmPot = 0.0
                Rate = 0.0
                
            #--------Bridge Manulipulator
                bridgePos = globVar.bridgeMax #inital position

            #---------Motor Initalizing
                setL = 0.0
                setR = 0.0
                setElevator = 0.0
                setFrontFeed = 0.0
                setBackFeed = 0.0
                setShooter = 0.0
                setShooterNeck = 0.0
                setBridge = 0.0

                while self.IsOperatorControl() and self.IsEnabled():
                        dog.Feed()
                        
        #-------------last Boolean Values
                        lastValues = functions.lastValueJoy(robotVar.stick, 12)
                        
        #---------------------Pause                
                        robotVar.Wait(0.01) #getting rid of this will cause the lastButton values to be difficult to use as they will miss a lot, and it will break the bot
        
        #---------------Setup Control over the Turret of the bot
                        setShooterNeck = functions.smartDeadZone(robotVar.stick.GetRawAxis(4), globVar.neckDeadZone, setShooterNeck)    
                        
        #------------------Bridge Control
                        #setBridge = smartDeadZone(robotVar.stick.GetRawAxis(5), globVar.neckDeadZone, setBridge)

                        #this makes a toggle function by comparing the last value to the current
                        if robotVar.stick.GetRawButton(7) and not lastValues[7]:
                            if bridgePos == globVar.bridgeMin:
                                bridgePos = globVar.bridgeMax
                            else:
                                bridgePos = globVar.bridgeMin

                        setBridge = functions.gotoPosition(robotVar.bridgePot.GetVoltage(), bridgePos, globVar.bridgePositionGain, globVar.bridgeStopWindow) * -1
        #--------------A simple drive with a arcade style drive
                        #setL = smartDeadZone(robotVar.stick.GetRawAxis(5), driveDeadZone, setL)
                        #setR = smartDeadZone(robotVar.stick.GetRawAxis(2), driveDeadZone, setR)
                        
                        setR, setL = functions.arcadeDrive(robotVar.stick.GetRawAxis(2), robotVar.stick.GetRawAxis(1), globVar.driveDeadZone)
        #---------------------Ball Feed System Control
                        if robotVar.stick.GetRawButton(1):
                            setElevator = 0.5
                        if robotVar.stick.GetRawButton(5):
                            setElevator = -0.5
                        if robotVar.stick.GetRawButton(6):
                            setFrontFeed = -1
                            setBackFeed = -1
                        if robotVar.stick.GetRawButton(4):
                            setFrontFeed = 1
                            setBackFeed = 1
                            setElevator = -0.5
                        
        #---------------------Get RPM (Non-Scaled)     
                        try:
                            #This tells us how much it has traveled since it was last looked at by comparing last to current
                            potDiff = (robotVar.rpmPot.GetVoltage() - lastRpmPot)

                            #The pots give some jumpy data so this is a very basic filter that seprates out any garbage in readings
                            if (abs(potDiff) < 0.02):
                                Rate = 0.0
                            else:
                                #here we actually calulate out the rate, rate is just distance travled over time, we time how fast the pot moved
                                #Becuase of the way the shooter works, we don't want negitive values
                                if (potDiff > 0):
                                    Rate = potDiff/robotVar.rpmTime.Get()
                                    Rate = (Rate + lastRate)/2
                                else:
                                    pass
                                if Rate > (globVar.maxRate * 1.01):
                                    Rate = lastRate
                        except ZeroDivisionError:
                            Rate = 0.0
                            print("RPM error zero divison")
                        except:
                            Rate = lastRate
                            print("RPM error unhaneled")
                        
                        robotVar.rpmTime.Reset()
                        lastRpmPot = robotVar.rpmPot.GetVoltage()
                        lastRate = Rate
        #-----------------------RPM control Code
                        wantedRate = functions.smartDeadZone(robotVar.stick.GetRawAxis(3), globVar.shooterDeadZone)
                        setShooter = functions.setRate(wantedRate, Rate, globVar.maxRate, setShooter, globVar.errorRateGain) * -1
                        #setShooter = smartDeadZone(stick.GetRawAxis(3), shooterDeadZone, setShooter)
        #--------------------Neck Saftey
                        if robotVar.neckEncode.Get() > globVar.shooterNeckLimit:
                            if setShooterNeck > 0:
                                setShooterNeck = 0
                        if robotVar.neckEncode.Get() < globVar.shooterNeckLimit*-1:
                            if setShooterNeck < 0:
                                setShooterNeck = 0
                        
        #------------------Bridge Safty
                        #This uses the bridge manulipluator's pot to make it so that if it trys to hyper extend out it will zero out the value
                        if robotVar.bridgePot.GetVoltage() > globVar.bridgeMax:
                            if setBridge > 0:
                                setBridge = 0
                        if robotVar.bridgePot.GetVoltage() < globVar.bridgeMin:
                            if setBridge < 0:
                                setBridge = 0
                                
        #-----------------Shooter Safty
                        #Shooter wheel will spin in only one direction, this makes sure of that
                        if setShooter > 0:
                            setShooter = 0

        #----------------Motor Safty
                        MortorVars = (setShooter, setShooterNeck, setBackFeed, setFrontFeed, setElevator, setBridge, setR, setL)
                        
                        (setShooter, setShooterNeck, setBackFeed, setFrontFeed, setElevator, setBridge, setR, setL) = functions.mortorSafty(MortorVars)

        #---------------- Motor control
                        robotVar.DriveL1.Set(setL)
                        robotVar.DriveL2.Set(setL)
                        robotVar.DriveR1.Set(setR*-1)
                        robotVar.DriveR2.Set(setR*-1)
                        
                        robotVar.bridgeWedge.Set(setBridge)
                        
                        robotVar.elevator.Set(setElevator)
                        robotVar.frontFeed.Set(setFrontFeed)
                        robotVar.backFeed.Set(setBackFeed)
                        robotVar.shooterRoller.Set(setShooter) #must be negative to shoot
                        robotVar.shooterNeck.Set(-1*setShooterNeck)
                        
        #-------------- Iteration Speed               
                        if robotVar.autonTimer.Get() > maxRateVal:
                            maxRateVal = robotVar.autonTimer.Get()
                        if robotVar.stick.GetRawButton(2):
                            maxRateVal = 0.0
                            
                        robotVar.autonTimer.Reset()
                        
        #-------------- Driverstaion LCD display
                        robotVar.ds.Clear()
                        
                        robotVar.ds.Print(robotVar.ds.Line(0), 1, "wantedRate: " + str(wantedRate * globVar.maxRate))
                        robotVar.ds.Print(robotVar.ds.Line(1), 1, "setRate:" + str(functions.setRate(wantedRate, Rate, globVar.maxRate, setShooter, globVar.errorRateGain) * -1))
                        robotVar.ds.Print(robotVar.ds.Line(2), 1, "rpmPot: " + str(robotVar.rpmPot.GetVoltage()))
                        robotVar.ds.Print(robotVar.ds.Line(3), 1, "Rate :" + str(Rate))
                        robotVar.ds.Print(robotVar.ds.Line(4), 1, "ThreadedRate: " + str(robotVar.rateOfPot.getRate()))
                        robotVar.ds.Print(robotVar.ds.Line(5), 1, "Iteration Speed: " + str(maxRateVal))
                        
                        robotVar.ds.UpdateLCD()
        #---------------------zero cause, not needed for rate speed setting
                        setL = 0.0
                        setR = 0.0
                        setElevator = 0.0
                        setFrontFeed = 0.0
                        setBackFeed = 0.0
                        #setShooter = 0.0
                        setShooterNeck = 0.0
                        setBridge = 0.0
                        
        

def run():
        robot = MyRobot()
        robot.StartCompetition()
