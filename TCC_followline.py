import RPi.GPIO as GPIO
import time
import cv2
import threading
from pyzbar import pyzbar
from pyzbar.pyzbar import decode
from picamera import PiCamera

#Definir pinos GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

sensor_esquerdo = 20
sensor_direito = 21
motor1A = 17
motor1B = 27
motor2A = 23
motor2B = 24
buzzer = 16
echo = 6
trig = 5
led_vermelho = 26
led_verde = 19
led_azul = 13

#Configurar pinos como INPUT ou OUPUT
GPIO.setup(sensor_esquerdo, GPIO.IN)
GPIO.setup(sensor_direito, GPIO.IN)
GPIO.setup(motor1A, GPIO.OUT)
GPIO.setup(motor1B, GPIO.OUT)
GPIO.setup(motor2A, GPIO.OUT)
GPIO.setup(motor2B, GPIO.OUT)
GPIO.setup(buzzer, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(led_vermelho, GPIO.OUT)
GPIO.setup(led_verde, GPIO.OUT)
GPIO.setup(led_azul, GPIO.OUT)

#Inicializar a câmera
camera = PiCamera()
camera.rotation = 180

#Configurar os pinos PWM para os motores em uma frequência
motor1A_pwm = GPIO.PWM(motor1A, 2000)
motor1B_pwm = GPIO.PWM(motor1B, 2000)
motor2A_pwm = GPIO.PWM(motor2A, 2000)
motor2B_pwm = GPIO.PWM(motor2B, 2000)

#Iniciar os motores com 0% de duty cycle(parado)
motor1A_pwm.start(0)
motor1B_pwm.start(0)
motor2A_pwm.start(0)
motor2B_pwm.start(0)

#Configurar o pino do buzzer como PWM
buzzer_pwm = GPIO.PWM(buzzer, 1000)

#Iniciar o buzzer com 0% de duty cycle(desligado)
buzzer_pwm.start(0)

seguir_linha_ativo = False

def parar():
	motor1A_pwm.ChangeDutyCycle(0)
	motor1B_pwm.ChangeDutyCycle(0)
	motor2A_pwm.ChangeDutyCycle(0)
	motor2B_pwm.ChangeDutyCycle(0)
	
def continuar():
	motor1A_pwm.ChangeDutyCycle(55)
	motor1B_pwm.ChangeDutyCycle(0)
	motor2A_pwm.ChangeDutyCycle(60)
	motor2B_pwm.ChangeDutyCycle(0)
	
#Função para girar 90 graus à esquerda
def girar_90_esquerda():
	motor1A_pwm.ChangeDutyCycle(50)
	motor1B_pwm.ChangeDutyCycle(0)
	motor2A_pwm.ChangeDutyCycle(0)
	motor2B_pwm.ChangeDutyCycle(60)
	time.sleep(0.6)
	motor1A_pwm.ChangeDutyCycle(0)
	motor1B_pwm.ChangeDutyCycle(0)
	motor2A_pwm.ChangeDutyCycle(0)
	motor2B_pwm.ChangeDutyCycle(0)
	
#Função para girar 90 graus à direita
def girar_90_direita():
	motor1A_pwm.ChangeDutyCycle(0)
	motor1B_pwm.ChangeDutyCycle(50)
	motor2A_pwm.ChangeDutyCycle(60)
	motor2B_pwm.ChangeDutyCycle(0)
	time.sleep(0.6)
	motor1A_pwm.ChangeDutyCycle(0)
	motor1B_pwm.ChangeDutyCycle(0)
	motor2A_pwm.ChangeDutyCycle(0)
	motor2B_pwm.ChangeDutyCycle(0)
	


#Função para seguir a linha
def seguir_linha():
	distancia = detectar_obstaculo()
	time.sleep(0.01)
	while seguir_linha_ativo and distancia >= 20:
		if GPIO.input(sensor_esquerdo) == GPIO.LOW and GPIO.input(sensor_direito) == GPIO.HIGH:
			# Sensor direito encostando na linha
			motor1A_pwm.ChangeDutyCycle(0)
			motor1B_pwm.ChangeDutyCycle(70)
			motor2A_pwm.ChangeDutyCycle(50)
			motor2B_pwm.ChangeDutyCycle(0)
		elif GPIO.input(sensor_esquerdo) == GPIO.HIGH and GPIO.input(sensor_direito) == GPIO.LOW:
			# Sensor esquerdo encostando na linha
			motor1A_pwm.ChangeDutyCycle(40)
			motor1B_pwm.ChangeDutyCycle(0)
			motor2A_pwm.ChangeDutyCycle(0)
			motor2B_pwm.ChangeDutyCycle(70)
		elif GPIO.input(sensor_esquerdo) == GPIO.LOW and GPIO.input(sensor_direito) == GPIO.LOW:
			# Ambos os sensores fora da linha (Em frente)
			motor1A_pwm.ChangeDutyCycle(34) ##direita
			motor1B_pwm.ChangeDutyCycle(0)
			motor2A_pwm.ChangeDutyCycle(40) ##esquerda
			motor2B_pwm.ChangeDutyCycle(0)
		else:
			parar()

		
#Função para detectar obstáculos com o sensor ultrassônico
def detectar_obstaculo():
	GPIO.output(trig, GPIO.HIGH)
	time.sleep(0.001)
	GPIO.output(trig, GPIO.LOW)
	
	while GPIO.input(echo) == 0:
		pulso_inicio = time.time()
		
	while GPIO.input(echo) == 1:
		pulso_fim = time.time()
	
	duracao = pulso_fim - pulso_inicio
	distancia = duracao * 17150 #velocidade do som (343 m/s) / 2
	
	return distancia
	
	
#Função para acionar o alarme do buzzer
def acionar_buzzer():
	buzzer_pwm.start(1)
	for _ in range(3):
		for i in range(100, 2000, 100):
			buzzer_pwm.ChangeFrequency(i)
			time.sleep(0.01)
		for i in range(2000, 100, -100):
			buzzer_pwm.ChangeFrequency(i)
			time.sleep(0.01)
	buzzer_pwm.ChangeDutyCycle(0)
	

#Função para ler os QR codes
def ler_qr_code():
	camera.capture("qr_code.jpg")
	
	# Carregar a imagem capturada
	image = cv2.imread("qr_code.jpg")
	
	# Converter a imagem para escala de cinza
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	
	# Detectar e decodificar os QR codes
	qr_codes = pyzbar.decode(gray)
	
	# Verificar se algum QR code foi detectado
	if qr_codes:
		qr_code_data = qr_codes[0].data.decode("utf-8")
		return qr_code_data
	
	return None

#Loop principal
try:
	while True:
		distancia = detectar_obstaculo()
		if distancia < 20:
			seguir_linha_ativo = False
			time.sleep(0.1)
			parar()
			acionar_buzzer()
		else:
			seguir_linha_ativo = True
			time.sleep(0.1)
		# Seguir a linha
			thread_seguir_linha = threading.Thread(target=seguir_linha)
			thread_seguir_linha.start()

		# Ler QR code
		qr_code = ler_qr_code()

		
		# Verificar ação a ser tomada com base no QR code lido
		if qr_code == "Standby":
			seguir_linha_ativo = False
			time.sleep(0.1)
			GPIO.output(led_vermelho, GPIO.HIGH)
			time.sleep(10)
			GPIO.output(led_vermelho, GPIO.LOW)
			motor1A_pwm.ChangeDutyCycle(0)
			motor1B_pwm.ChangeDutyCycle(50)
			motor2A_pwm.ChangeDutyCycle(60)
			motor2B_pwm.ChangeDutyCycle(0)
			time.sleep(0.1)
			continuar()
			time.sleep(0.15)
			seguir_linha_ativo = True
			time.sleep(0.1)
		elif qr_code == "Carga":
			seguir_linha_ativo = False
			time.sleep(0.1)
			GPIO.output(led_azul, GPIO.HIGH)
			time.sleep(0.1)
			girar_90_esquerda()
			time.sleep(5)
			girar_90_direita()
			time.sleep(0.1)
			GPIO.output(led_azul, GPIO.LOW)
			continuar()
			time.sleep(0.2)
			seguir_linha_ativo = True
			time.sleep(0.1)
		elif qr_code == "Descarga":
			seguir_linha_ativo = False
			time.sleep(0.1)
			GPIO.output(led_verde, GPIO.HIGH)
			time.sleep(5)
			GPIO.output(led_verde, GPIO.LOW)
			continuar()
			time.sleep(0.2)
			seguir_linha_ativo = True
			time.sleep(0.1)


except KeyboardInterrupt:
	GPIO.cleanup()
