#!/usr/bin/env python3
import math

# Parámetros del movimiento (Tramo Rojo - Ida)
z_inicial = 0.250
z_final = 0.107
h = z_final - z_inicial  # Distancia negativa (-0.143 m)
dist = abs(h)

# Límites de la consultora (Tramo Rojo)
v_max_lim = 0.200
a_max_lim = 0.300

print("=== ANÁLISIS DE TIEMPO DE VUELO (tf) ===")
# Cálculo tf para Cúbica
tf_cub_v = (3 * dist) / (2 * v_max_lim)
tf_cub_a = math.sqrt((6 * dist) / a_max_lim)
tf_min_cub = max(tf_cub_v, tf_cub_a)

# Cálculo tf para Quíntica
tf_qui_v = (15 * dist) / (8 * v_max_lim)
tf_qui_a = math.sqrt((5.7735 * dist) / a_max_lim)
tf_min_qui = max(tf_qui_v, tf_qui_a)

print(f"Tiempo mínimo Cúbica:   {tf_min_cub:.3f} s")
print(f"Tiempo mínimo Quíntica: {tf_min_qui:.3f} s")

# Escogemos un tiempo seguro para ambas (redondeado hacia arriba)
tf = 2.0 
print(f"-> Tiempo de vuelo SELECCIONADO: {tf} s\n")

# Coeficientes Cúbicos
c_a0 = z_inicial
c_a1 = 0
c_a2 = (3 * h) / (tf**2)
c_a3 = (-2 * h) / (tf**3)

# Coeficientes Quínticos
q_a0 = z_inicial
q_a1 = 0
q_a2 = 0
q_a3 = (10 * h) / (tf**3)
q_a4 = (-15 * h) / (tf**4)
q_a5 = (6 * h) / (tf**5)

print("=== PERFILES TEMPORALES (Posición, Velocidad, Aceleración) ===")
print("Format: [Tiempo] | Posición (Z) | Velocidad (V) | Aceleración (A)")

# 3 puntos intermedios + inicio + fin = 5 instantes
tiempos = [0.0, 0.5, 1.0, 1.5, 2.0]

print("\n--- INTERPOLACIÓN CÚBICA ---")
for t in tiempos:
    pos = c_a0 + c_a1*t + c_a2*(t**2) + c_a3*(t**3)
    vel = c_a1 + 2*c_a2*t + 3*c_a3*(t**2)
    acc = 2*c_a2 + 6*c_a3*t
    print(f"t={t:.1f}s | P: {pos:.4f} m | V: {vel:.4f} m/s | A: {acc:.4f} m/s²")

print("\n--- INTERPOLACIÓN QUÍNTICA ---")
for t in tiempos:
    pos = q_a0 + q_a1*t + q_a2*(t**2) + q_a3*(t**3) + q_a4*(t**4) + q_a5*(t**5)
    vel = q_a1 + 2*q_a2*t + 3*q_a3*(t**2) + 4*q_a4*(t**3) + 5*q_a5*(t**4)
    acc = 2*q_a2 + 6*q_a3*t + 12*q_a4*(t**2) + 20*q_a5*(t**3)
    print(f"t={t:.1f}s | P: {pos:.4f} m | V: {vel:.4f} m/s | A: {acc:.4f} m/s²")
