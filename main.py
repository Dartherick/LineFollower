import sim
import numpy as np
from time import sleep

class LineFollower():
	def __init__(self,Port):
		'''
		Para enlazar coppelia sim con el codigo realizado en python,coloca la siguiente funcion en un script en coppelia
			simExtRemoteApiStart(Port)
		Donde Port tiene que ser exactamente el mismo colocado en python
		'''
		sim.simxFinish(-1) #Cerrando cualquier comunicacion previamente abierta
		self.clientID = sim.simxStart('127.0.0.1',Port,True,True,5000,5) #Conectando coppelia con el codigo

		if self.clientID!=-1:
			print ("Connected to remote API server")
			sim.simxStartSimulation(self.clientID,sim.simx_opmode_oneshot)
		else:
			print("Not connected to remote API server")
			exit()

	def Handles(self,Car,LeftMotor,RightMotor,LeftSensor,RightSensor,BColorSensor,BAproxSensor): #Funcion para obtener los handles
		self.Car = sim.simxGetObjectHandle(self.clientID, Car, sim.simx_opmode_blocking)[-1]						#Cuerpo del carro
		self.LMotor = sim.simxGetObjectHandle(self.clientID, LeftMotor, sim.simx_opmode_blocking)[-1]				#Motor izquierdo
		self.RMotor = sim.simxGetObjectHandle(self.clientID, RightMotor, sim.simx_opmode_blocking)[-1]				#Motor derecho
		self.LSensor = sim.simxGetObjectHandle(self.clientID, LeftSensor, sim.simx_opmode_blocking)[-1]				#Sensor de vision izquierdo
		self.RSensor = sim.simxGetObjectHandle(self.clientID, RightSensor, sim.simx_opmode_blocking)[-1]			#Sensor de vision derecho
		self.BColorSensor = sim.simxGetObjectHandle(self.clientID, BColorSensor, sim.simx_opmode_blocking)[-1]		#Sensor de vision para detectar el color del bloque
		self.BAproxSensor = sim.simxGetObjectHandle(self.clientID, BAproxSensor, sim.simx_opmode_blocking)[-1]		#Sensor de vision para detectar el bloque
		self.__InitalizeSensors()

	def __InitalizeSensors(self):
		'''Funcion para inicializar los sensores de vision
		se necesitan como minimo 0.02 segundos entre el modo streaming y buffer'''

		#Sensores de vision para detectar las lineas
		sim.simxGetVisionSensorImage(self.clientID,self.LSensor,0,sim.simx_opmode_streaming)[-1]
		sim.simxGetVisionSensorImage(self.clientID,self.RSensor,0,sim.simx_opmode_streaming)[-1]

		#Sensores para detectar el bloque
		sim.simxGetVisionSensorImage(self.clientID,self.BColorSensor,0,sim.simx_opmode_streaming)[-1]
		sim.simxReadProximitySensor(self.clientID,self.BAproxSensor,sim.simx_opmode_streaming)[-1]
		sleep(0.02)

	def LineFollow(self,Velocity,X):
		#Leyendo los datos obtenidos con los sensores y almacenandolos en variables
		
		LeftImageData = self.__Normalize(sim.simxGetVisionSensorImage(self.clientID,self.LSensor,0,sim.simx_opmode_buffer)[-1])
		RightImageData = self.__Normalize(sim.simxGetVisionSensorImage(self.clientID,self.RSensor,0,sim.simx_opmode_buffer)[-1])
		print(f'{LeftImageData} {RightImageData}')

		'''
		Negro = 0 ; Blanco = 1
		_______________________
		LMotor|RMotor|Direccion
		   0	 0	 |	  ↑
		   0	 1	 |	  →
		   1	 0	 |	  ←
		   1	 1	 |	  ↓
		   -	 -	 |	Error
		'''
		
		if not LeftImageData and not RightImageData:
			#Left & Right motor = 0
			sim.simxSetJointTargetVelocity(self.clientID,self.LMotor,Velocity,sim.simx_opmode_streaming)
			sim.simxSetJointTargetVelocity(self.clientID,self.RMotor,Velocity,sim.simx_opmode_streaming)

		elif not LeftImageData and RightImageData:
			#Left motor = 0 & Right motor = 1
			sim.simxSetJointTargetVelocity(self.clientID,self.LMotor,Velocity*X,sim.simx_opmode_streaming)
			sim.simxSetJointTargetVelocity(self.clientID,self.RMotor,Velocity,sim.simx_opmode_streaming)

		elif LeftImageData and not RightImageData:
			#Left motor = 1 & Right motor = 0
			sim.simxSetJointTargetVelocity(self.clientID,self.LMotor,Velocity,sim.simx_opmode_streaming)
			sim.simxSetJointTargetVelocity(self.clientID,self.RMotor,Velocity*X,sim.simx_opmode_streaming)

	def __Normalize(self,RGB):
		if all(i>0 for i in RGB):
			try:
				return round((sum(RGB)/len(RGB))/255)
			except:
				None
		else:
			return 1

	def DetectBlock(self):
		if sim.simxReadProximitySensor(self.clientID,self.BAproxSensor,sim.simx_opmode_buffer)[1]:
			self.__DetectColorBlock()
			return True
		else:
			return False

	def __DetectColorBlock(self):
		sim.simxSetJointTargetVelocity(self.clientID,self.LMotor,0,sim.simx_opmode_streaming)
		sim.simxSetJointTargetVelocity(self.clientID,self.RMotor,0,sim.simx_opmode_streaming)

		RGB = sim.simxGetVisionSensorImage(self.clientID,self.BColorSensor,0,sim.simx_opmode_buffer)[-1]
		print(sim.simxGetVisionSensorImage(self.clientID,self.BColorSensor,0,sim.simx_opmode_buffer))
	    
		Colors = {'Rojo':(255,0,0),'Verde':(0,255,0),'Azul':(0,0,255),'Amarillo':(255,255,0),'Morado':(128,0,128),'Naranja':(255,165,0),'Rosado':(255,192,203),'Negro':(0,0,0),'Blanco':(255,255,255),'Gris':(128,128,128),'Violeta':(238,130,238)}

		ColorList = np.array(list(Colors.values()))

		Edist = (np.sum((ColorList - RGB)**2,axis=1))**(1/2)
		Index = np.argmin(Edist)
		ColorBlock = 'El color del bloque es: ' + list(Colors.keys())[Index]
		
		self.__CallScript()

		DialogHandle = sim.simxDisplayDialog(self.clientID,'Color',ColorBlock,sim.sim_dlgstyle_message,'',None,RGB,sim.simx_opmode_blocking)[1]
		sleep(5)
		sim.simxEndDialog(self.clientID,DialogHandle,sim.simx_opmode_oneshot)
		sim.simxStopSimulation(self.clientID,sim.simx_opmode_oneshot)
		exit()
		
	def __CallScript(self):
		sim.simxCallScriptFunction(self.clientID,'',sim.sim_scripttype_childscript (1),'RoboticHand',[],[],[],bytearray(),sim.simx_opmode_blocking)

Car = LineFollower(2429)
Car.Handles('robot_cuerpo','LeftMotor','RightMotor','LeftSensor','RightSensor','BColorSensor','BAproxSensor')
Velocity = 8
X = 0.7

while not Car.DetectBlock():
	Car.LineFollow(Velocity,X)