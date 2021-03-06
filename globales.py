
# marcial PID INICIALES - pixeles (x-y) - cm (z) - degree (head)
global KPx, KPy, KPz, KPangle, KIx, KIy, KIz, KIangle, KDx, KDy, KDz, KDangle
global throttle_middle, aileron_middle, elevator_middle, rudder_middle, correccion_gravedad, clamp_offset
global AUXILIAR1, KPzd, KIzd, KDzd

# commandos
global throttle, aileron, elevator, rudder

global fps, timerStart, config_activa, info
global refresca_params_flag
global midron

# datos del localizador  # provisional!!!
global x, y, z, head

# objetivo
global xTarget, yTarget, zTarget, angleTarget

# provisional
global kalman_angle, disable_all_kalmans, solo_buscar_en_cercanias
global traza_aux

global frame, frame_time     # frame_time se puede usar para tratar el tema PID.

global avanza, valor_movimiento, retrocede

global path_x1, path_y1, path_x2, path_y2, path_x3, path_y3
global fp1, fp2, fp3, path_x, path_y, path_angle


global manual_thr, manual_ail, manual_ele
first_frame = None
aux_debug = False