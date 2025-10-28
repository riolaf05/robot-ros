#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import time
import re
from hardware_interface import SystemInterface, HardwareInfo, CallbackReturn
from rclpy.duration import Duration


class ArduinoHardwareInterface(SystemInterface):
    """
    Hardware interface per comunicare con Arduino utilizzando il protocollo
    del ros_arduino_bridge firmware.
    """
    
    def __init__(self):
        super().__init__()
        self.arduino = None
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.left_wheel_cmd = 0.0
        self.right_wheel_cmd = 0.0
        
        # Parametri del robot
        self.wheel_radius = 0.065  # m
        self.wheel_separation = 0.17  # m
        self.ticks_per_rev = 3436
        
        # Comunicazione seriale
        self.port = "/dev/ttyUSB0"
        self.baudrate = 57600
        self.timeout = 1.0
        
        # Thread per lettura encoder
        self.reading_thread = None
        self.is_running = False
        
        # Buffer per i comandi
        self.command_lock = threading.Lock()
        
    def on_init(self, info: HardwareInfo) -> CallbackReturn:
        """Inizializzazione dell'interfaccia hardware"""
        
        # Leggi i parametri dal file URDF
        try:
            if "device" in info.hardware_parameters:
                self.port = info.hardware_parameters["device"]
            if "baud_rate" in info.hardware_parameters:
                self.baudrate = int(info.hardware_parameters["baud_rate"])
            if "timeout" in info.hardware_parameters:
                self.timeout = float(info.hardware_parameters["timeout"]) / 1000.0
            if "enc_counts_per_rev" in info.hardware_parameters:
                self.ticks_per_rev = int(info.hardware_parameters["enc_counts_per_rev"])
                
        except Exception as e:
            self.get_logger().error(f"Error reading parameters: {e}")
            return CallbackReturn.ERROR
            
        # Verifica che i joint siano corretti
        if len(info.joints) != 2:
            self.get_logger().error("Expected exactly 2 joints")
            return CallbackReturn.ERROR
            
        self.left_joint_name = info.joints[0].name
        self.right_joint_name = info.joints[1].name
        
        self.get_logger().info(f"Arduino Hardware Interface initialized")
        self.get_logger().info(f"Port: {self.port}, Baudrate: {self.baudrate}")
        self.get_logger().info(f"Joints: {self.left_joint_name}, {self.right_joint_name}")
        
        return CallbackReturn.SUCCESS
        
    def on_configure(self, previous_state) -> CallbackReturn:
        """Configurazione dell'interfaccia"""
        
        try:
            # Connetti ad Arduino
            self.arduino = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            
            # Aspetta che Arduino si inizializzi
            time.sleep(2)
            
            # Pulisci il buffer
            self.arduino.flushInput()
            self.arduino.flushOutput()
            
            # Testa la connessione
            self.send_command("v")  # Richiedi versione
            time.sleep(0.1)
            
            # Leggi la risposta
            response = self.read_response()
            if response:
                self.get_logger().info(f"Arduino connected: {response}")
            else:
                self.get_logger().warn("No response from Arduino, continuing anyway...")
                
            # Reset encoders
            self.send_command("r")
            
            self.get_logger().info("Arduino configured successfully")
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Failed to configure Arduino: {e}")
            return CallbackReturn.ERROR
            
    def on_activate(self, previous_state) -> CallbackReturn:
        """Attivazione dell'interfaccia"""
        
        try:
            # Avvia il thread di lettura
            self.is_running = True
            self.reading_thread = threading.Thread(target=self.read_encoders_loop)
            self.reading_thread.daemon = True
            self.reading_thread.start()
            
            self.get_logger().info("Arduino Hardware Interface activated")
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Failed to activate Arduino interface: {e}")
            return CallbackReturn.ERROR
            
    def on_deactivate(self, previous_state) -> CallbackReturn:
        """Disattivazione dell'interfaccia"""
        
        # Ferma i motori
        self.send_motor_commands(0, 0)
        
        # Ferma il thread di lettura
        self.is_running = False
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=1.0)
            
        self.get_logger().info("Arduino Hardware Interface deactivated")
        return CallbackReturn.SUCCESS
        
    def on_cleanup(self, previous_state) -> CallbackReturn:
        """Pulizia dell'interfaccia"""
        
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
            
        self.get_logger().info("Arduino Hardware Interface cleaned up")
        return CallbackReturn.SUCCESS
        
    def on_error(self, previous_state) -> CallbackReturn:
        """Gestione errori"""
        
        self.get_logger().error("Arduino Hardware Interface in error state")
        return CallbackReturn.SUCCESS
        
    def read(self, time, period) -> bool:
        """Lettura dello stato del hardware"""
        # I dati vengono aggiornati dal thread di lettura
        return True
        
    def write(self, time, period) -> bool:
        """Scrittura dei comandi al hardware"""
        
        try:
            with self.command_lock:
                # Converti velocità angolare delle ruote in PWM
                left_pwm = self.velocity_to_pwm(self.left_wheel_cmd)
                right_pwm = self.velocity_to_pwm(self.right_wheel_cmd)
                
                # Invia comandi ai motori
                self.send_motor_commands(left_pwm, right_pwm)
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error in write: {e}")
            return False
            
    def send_command(self, command):
        """Invia un comando ad Arduino"""
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write((command + '\r').encode())
                self.arduino.flush()
            except Exception as e:
                self.get_logger().error(f"Error sending command '{command}': {e}")
                
    def read_response(self):
        """Leggi una risposta da Arduino"""
        if self.arduino and self.arduino.is_open:
            try:
                response = self.arduino.readline().decode().strip()
                return response
            except Exception as e:
                self.get_logger().error(f"Error reading response: {e}")
        return None
        
    def send_motor_commands(self, left_pwm, right_pwm):
        """Invia comandi PWM ai motori"""
        # Limita i valori PWM
        left_pwm = max(-255, min(255, int(left_pwm)))
        right_pwm = max(-255, min(255, int(right_pwm)))
        
        # Formato comando: m left_pwm right_pwm
        command = f"m {left_pwm} {right_pwm}"
        self.send_command(command)
        
    def read_encoders_loop(self):
        """Loop per leggere gli encoder in un thread separato"""
        
        while self.is_running:
            try:
                if self.arduino and self.arduino.is_open:
                    # Richiedi lettura encoder
                    self.send_command("e")
                    
                    # Leggi risposta
                    response = self.read_response()
                    
                    if response:
                        # Parsa la risposta degli encoder
                        # Formato atteso: "left_ticks right_ticks"
                        parts = response.split()
                        if len(parts) >= 2:
                            try:
                                left_ticks = int(parts[0])
                                right_ticks = int(parts[1])
                                
                                # Converti ticks in posizione (radianti)
                                left_pos = (left_ticks / self.ticks_per_rev) * 2 * 3.14159
                                right_pos = (right_ticks / self.ticks_per_rev) * 2 * 3.14159
                                
                                # Calcola velocità (differenza dalla lettura precedente)
                                dt = 0.1  # 10 Hz
                                left_vel = (left_pos - self.left_wheel_pos) / dt
                                right_vel = (right_pos - self.right_wheel_pos) / dt
                                
                                # Aggiorna i valori
                                self.left_wheel_pos = left_pos
                                self.right_wheel_pos = right_pos
                                self.left_wheel_vel = left_vel
                                self.right_wheel_vel = right_vel
                                
                            except ValueError:
                                self.get_logger().warn(f"Invalid encoder response: {response}")
                                
                time.sleep(0.1)  # 10 Hz
                
            except Exception as e:
                self.get_logger().error(f"Error in encoder reading loop: {e}")
                time.sleep(1.0)
                
    def velocity_to_pwm(self, velocity):
        """Converti velocità angolare (rad/s) in PWM (-255 a 255)"""
        # Fattore di conversione empirico
        # Potrebbe essere necessario calibrarlo per il tuo robot
        max_velocity = 10.0  # rad/s
        pwm = (velocity / max_velocity) * 255
        return max(-255, min(255, pwm))
        
    def export_state_interfaces(self):
        """Esporta le interfacce di stato"""
        state_interfaces = []
        
        # Interfacce per la ruota sinistra
        state_interfaces.append(
            HardwareInterface(
                name=f"{self.left_joint_name}/position",
                initial_value=0.0
            )
        )
        state_interfaces.append(
            HardwareInterface(
                name=f"{self.left_joint_name}/velocity", 
                initial_value=0.0
            )
        )
        
        # Interfacce per la ruota destra
        state_interfaces.append(
            HardwareInterface(
                name=f"{self.right_joint_name}/position",
                initial_value=0.0
            )
        )
        state_interfaces.append(
            HardwareInterface(
                name=f"{self.right_joint_name}/velocity",
                initial_value=0.0
            )
        )
        
        return state_interfaces
        
    def export_command_interfaces(self):
        """Esporta le interfacce di comando"""
        command_interfaces = []
        
        # Interfacce per la ruota sinistra
        command_interfaces.append(
            HardwareInterface(
                name=f"{self.left_joint_name}/velocity",
                initial_value=0.0
            )
        )
        
        # Interfacce per la ruota destra  
        command_interfaces.append(
            HardwareInterface(
                name=f"{self.right_joint_name}/velocity",
                initial_value=0.0
            )
        )
        
        return command_interfaces
        
    def get_state_interface_value(self, interface_name):
        """Ottieni il valore di un'interfaccia di stato"""
        if "left_wheel_joint/position" in interface_name:
            return self.left_wheel_pos
        elif "left_wheel_joint/velocity" in interface_name:
            return self.left_wheel_vel
        elif "right_wheel_joint/position" in interface_name:
            return self.right_wheel_pos
        elif "right_wheel_joint/velocity" in interface_name:
            return self.right_wheel_vel
        return 0.0
        
    def set_command_interface_value(self, interface_name, value):
        """Imposta il valore di un'interfaccia di comando"""
        if "left_wheel_joint/velocity" in interface_name:
            self.left_wheel_cmd = value
        elif "right_wheel_joint/velocity" in interface_name:
            self.right_wheel_cmd = value