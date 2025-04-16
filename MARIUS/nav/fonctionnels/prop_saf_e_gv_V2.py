import RPi.GPIO as GPIO
from MCP3008 import MCP3008
import math

# Désactivation des avertissements GPIO
GPIO.setwarnings(False)

# Configuration GPIO
GPIO.setmode(GPIO.BOARD)

# Pins Grand-Voile (GV)
ENA_GV = 32
SENS_GV = 29
GPIO.setup(ENA_GV, GPIO.OUT)
GPIO.setup(SENS_GV, GPIO.OUT)

# Pins Safran
ENA_SAF = 33
SENS_SAF = 31
GPIO.setup(ENA_SAF, GPIO.OUT)
GPIO.setup(SENS_SAF, GPIO.OUT)

# Initialisation PWM
PWM_GV = GPIO.PWM(ENA_GV, 100)
PWM_GV.start(0)
PWM_SAF = GPIO.PWM(ENA_SAF, 100)
PWM_SAF.start(0)

# Convertisseur ADC
adc = MCP3008()

# Configuration GV 
GV_LEFT_ADC = 640    # -30°
GV_CENTER_ADC = 760  # 0°
GV_RIGHT_ADC = 870   # +30°
GV_MIN_PWM = 50
GV_KP = 20.0

# Configuration Safran
SAF_LEFT_ADC = 500    # -45°
SAF_CENTER_ADC = 770  # 0°
SAF_RIGHT_ADC = 1000  # +45°
SAF_MIN_PWM = 30
SAF_KP = 3.0

def adc_to_gv_angle(adc_value):
    """Conversion ADC → angle GV en radians"""
    adc_value = max(GV_LEFT_ADC, min(adc_value, GV_RIGHT_ADC))
    if adc_value >= GV_CENTER_ADC:
        return -((adc_value - GV_CENTER_ADC) / (GV_RIGHT_ADC - GV_CENTER_ADC)) * (math.pi/6)
    else:
        return -((adc_value - GV_CENTER_ADC) / (GV_CENTER_ADC - GV_LEFT_ADC)) * (math.pi/6)

def adc_to_saf_angle(adc_value):
    """Conversion ADC → angle SAF en radians"""
    adc_value = max(SAF_LEFT_ADC, min(adc_value, SAF_RIGHT_ADC))
    if adc_value >= SAF_CENTER_ADC:
        return ((adc_value - SAF_CENTER_ADC) / (SAF_RIGHT_ADC - SAF_CENTER_ADC)) * (math.pi/4)
    else:
        return ((adc_value - SAF_CENTER_ADC) / (SAF_CENTER_ADC - SAF_LEFT_ADC)) * (math.pi/4)

def control_saf(desired_rad):
    """Contrôle proportionnel du safran"""
    adc_value = max(SAF_LEFT_ADC, min(adc.read(channel=1), SAF_RIGHT_ADC))
    current_rad = adc_to_saf_angle(adc_value)
    error = desired_rad - current_rad
    
    pwm = min(abs(error * SAF_KP), 100)
    pwm = max(pwm, SAF_MIN_PWM) if pwm > 0 else 0
    
    if error > 0.01:
        GPIO.output(SENS_SAF, GPIO.LOW)
        PWM_SAF.ChangeDutyCycle(pwm)
    elif error < -0.01:
        GPIO.output(SENS_SAF, GPIO.HIGH)
        PWM_SAF.ChangeDutyCycle(pwm)
    else:
        PWM_SAF.ChangeDutyCycle(0)
    
    return current_rad, pwm

def control_gv(desired_rad):
    """Contrôle proportionnel de la grand-voile"""
    adc_value = max(GV_LEFT_ADC, min(adc.read(channel=0), GV_RIGHT_ADC))
    current_rad = adc_to_gv_angle(adc_value)
    error = desired_rad - current_rad
    
    pwm = min(abs(error * GV_KP), 100)
    pwm = max(pwm, GV_MIN_PWM) if pwm > 0 else 0
    
    if error > 0.01:
        GPIO.output(SENS_GV, GPIO.HIGH)
        PWM_GV.ChangeDutyCycle(pwm)
    elif error < -0.01:
        GPIO.output(SENS_GV, GPIO.LOW)
        PWM_GV.ChangeDutyCycle(pwm)
    else:
        PWM_GV.ChangeDutyCycle(0)
    
    return current_rad, pwm

def cleanup():
    """Nettoyage des ressources GPIO"""
    PWM_GV.stop()
    PWM_SAF.stop()
    GPIO.cleanup()