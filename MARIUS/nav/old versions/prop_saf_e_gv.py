import RPi.GPIO as GPIO
from MCP3008 import MCP3008
import time
import math

# Configuration GPIO
GPIO.setmode(GPIO.BOARD)

# Pins Grand-Voile (GV)
ENA_GV = 32
SENS_GV = 29
GPIO.setup(ENA_GV, GPIO.OUT)
GPIO.setup(SENS_GV, GPIO.OUT)

# safran
ENA_SAF = 33  # Pin PWM
SENS_SAF = 31  # Pin sens
GPIO.setup(ENA_SAF, GPIO.OUT)
GPIO.setup(SENS_SAF, GPIO.OUT)

# Initialisation PWM
PWM_GV = GPIO.PWM(ENA_GV, 100)
PWM_GV.start(0)
PWM_SAF = GPIO.PWM(ENA_SAF, 100)
PWM_SAF.start(0)
# Convertisseur ADC
adc = MCP3008()


# Configuration GV (avec vos valeurs réelles)
GV_LEFT_ADC = 640    # -30° (valeur ADC minimale)
GV_CENTER_ADC = 740  # 0° (valeur ADC médiane)
GV_RIGHT_ADC = 870   # +30° (valeur ADC maximale)
GV_MIN_PWM = 50
GV_KP = 2.0  # Gain réduit pour plus de stabilité


#config pour le safran
SAF_LEFT_ADC = 510    # -45° (valeur ADC minimale)
SAF_CENTER_ADC = 770  # 0° (valeur ADC médiane)
SAF_RIGHT_ADC = 1010   # +45° (valeur ADC maximale)
SAF_MIN_PWM = 20
SAF_KP = 1.6  # Gain réduit pour plus de stabilité


def adc_to_gv_angle(adc_value):
    """Conversion ADC → angle GV en radians avec gestion correcte du sens"""
    if adc_value >= GV_CENTER_ADC:  # Côté droit (angles positifs)
        return ((adc_value - GV_CENTER_ADC) / (GV_RIGHT_ADC - GV_CENTER_ADC)) * (math.pi/6)
    else:  # Côté gauche (angles négatifs)
        return ((adc_value - GV_CENTER_ADC) / (GV_CENTER_ADC - GV_LEFT_ADC)) * (math.pi/6)

def adc_to_saf_angle(adc_value):
    """Conversion ADC → angle SAF en radians avec gestion correcte du sens"""
    if adc_value >= SAF_CENTER_ADC:  # Côté droit (angles positifs)
        return ((adc_value - SAF_CENTER_ADC) / (SAF_RIGHT_ADC - SAF_CENTER_ADC)) * (math.pi/4)
    else:  # Côté gauche (angles négatifs)
        return ((adc_value - SAF_CENTER_ADC) / (SAF_CENTER_ADC - SAF_LEFT_ADC)) * (math.pi/4)
    

def print_status_gv(target, current, error, pwm):
    """Affiche les infos de la GV"""
    print(f"GV  | Target: {math.degrees(target):6.1f}° | Current: {math.degrees(current):6.1f}° | Error: {math.degrees(error):6.1f}° | PWM: {pwm:3.0f}%")

def print_status_saf(target, current, error, pwm):
    """Affiche les infos du safran""" 
    print(f"SAF | Target: {math.degrees(target):6.1f}° | Current: {math.degrees(current):6.1f}° | Error: {math.degrees(error):6.1f}° | PWM: {pwm:3.0f}%")


def control_saf(desired_rad):
    """Contrôle proportionnel corrigé de la SAF"""
    adc_value_saf = adc.read(channel=1)
    current_rad_saf = adc_to_saf_angle(adc_value_saf)
    error_saf = desired_rad - current_rad_saf
    
    # Calcul du PWM proportionnel
    pwm_saf = min(abs(error_saf * SAF_KP), 100)
    pwm_saf = max(pwm_saf, SAF_MIN_PWM) if pwm_saf > 0 else 0
    
    # Détermination du sens (inversé si nécessaire)
    if error_saf > 0.01:  # Besoin d'aller vers la droite
        GPIO.output(SENS_SAF, GPIO.LOW)  # Ajustez ce sens si nécessaire
        PWM_SAF.ChangeDutyCycle(pwm_saf)
    elif error_saf < -0.01:  # Besoin d'aller vers la gauche
        GPIO.output(SENS_SAF, GPIO.HIGH)  # Ajustez ce sens si nécessaire
        PWM_SAF.ChangeDutyCycle(pwm_saf)
    else:
        PWM_SAF.ChangeDutyCycle(0)
    
    return current_rad_saf

def control_gv(desired_rad):
    """Contrôle proportionnel corrigé de la GV"""
    adc_value = adc.read(channel=0)
    current_rad = adc_to_gv_angle(adc_value)
    error = desired_rad - current_rad
    
    # Calcul du PWM proportionnel
    pwm = min(abs(error * GV_KP), 100)
    pwm = max(pwm, GV_MIN_PWM) if pwm > 0 else 0
    
    # Détermination du sens (inversé si nécessaire)
    if error > 0.01:  # Besoin d'aller vers la droite
        GPIO.output(SENS_GV, GPIO.LOW)  # Ajustez ce sens si nécessaire
        PWM_GV.ChangeDutyCycle(pwm)
    elif error < -0.01:  # Besoin d'aller vers la gauche
        GPIO.output(SENS_GV, GPIO.HIGH)  # Ajustez ce sens si nécessaire
        PWM_GV.ChangeDutyCycle(pwm)
    else:
        PWM_GV.ChangeDutyCycle(0)
    
    return current_rad

try:
    print("Contrôle proportionnel GV")
    print(f"Plage: -30° à +30° (ADC: {GV_LEFT_ADC}-{GV_RIGHT_ADC}, Centre: {GV_CENTER_ADC})")
    print("Contrôle proportionnel GV")
    print(f"Plage: -45° à +45° (ADC: {SAF_LEFT_ADC}-{SAF_RIGHT_ADC}, Centre: {SAF_CENTER_ADC})")
    while True:
        try:
            # Saisie utilisateur
            gv_angle = float(input("\nAngle GV en radians (-0.52 à +0.52): "))
            gv_angle = max(-math.pi/6, min(gv_angle, math.pi/6))  # Validation

            saf_angle = float(input("Entrez l'angle souhaité en radians (-0.78 à +0.78): "))
            saf_angle = max(-math.pi/4, min(saf_angle, math.pi/4))  # Validation
            
            print("\nContrôle actif (Ctrl+C pour arrêter)")
            print("------------------------------------")
            
            start_time = time.time()


            while True:
                current_gv = control_gv(gv_angle)
                current_saf = control_saf(saf_angle)

                print_status_gv(gv_angle, current_gv, gv_angle-current_gv, PWM_GV.get_duty_cycle())
                print_status_saf(saf_angle, current_saf, saf_angle-current_saf, PWM_SAF.get_duty_cycle())

                # Condition de sortie
                if abs(current_gv - gv_angle) < math.radians(1.0):
                    print("\nPosition gv atteinte!")
                    PWM_GV.ChangeDutyCycle(0)
                    break
                if abs(current_saf - saf_angle) < math.radians(1.0):
                    print("\nPosition saf atteinte!")
                    PWM_SAF.ChangeDutyCycle(0)
                    break
                time.sleep(0.1)
                
        except ValueError:
            print("Erreur: Entrez un nombre valide")

except KeyboardInterrupt:
    PWM_GV.stop()
    PWM_SAF.stop()
    GPIO.cleanup()
    print("\nArrêt propre du système")