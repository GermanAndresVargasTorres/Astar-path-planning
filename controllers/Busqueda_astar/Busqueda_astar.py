from controller import Robot, Supervisor, Node, Field
import math, sys, struct


# Clase para los objetos de tipo Celda
class Celda () :

  # ATRIBUTOS
  # Coordenadas en la grid de la celda
  coord_x = 1
  coord_y = 1       
  # Coordenadas en la grid de la celda padre
  coord_x_padre = 1
  coord_y_padre = 1        
  # Costo estimado desde este nodo hasta la meta
  valor_funcion_h = 0        
  # Costo del camino hasta este nodo
  valor_funcion_g = 0        
  # Funcion heuristica: f(n) = g(n) + h(n)
  valor_funcion_f = 0

  # CONSTRUCTOR
  def __init__(self, cx, cy, cxp, cyp, g, h, f):
    self.coord_x = cx
    self.coord_y = cy            
    self.coord_x_padre = cxp
    self.coord_y_padre = cyp
    self.valor_funcion_h = h
    self.valor_funcion_g = g
    self.valor_funcion_f = f


# Clase del controlador
class Busqueda_astar (Supervisor) :
  
  # Funcion que realiza la configuracion inicial, ejecuta la busqueda A* y comanda el robot EPuck
  def Ejecutar(self):
    
    # Obtener informacion de los nodos presentes en la simulacion
    entorno = self.getFromDef('Mundo')
    epuck = self.getFromDef('EPuck')
    bola = self.getFromDef('Ball')
    obs1 = self.getFromDef('Box1')
    obs2 = self.getFromDef('Box2')
    obs3 = self.getFromDef('Box3')
    obs4 = self.getFromDef('Box4')
    num_obstaculos = 4    
    
    # Crear variable para el nodo emisor (comunicacion inalambrica)
    emisor = self.getEmitter('emitter')
    
    # Crear una matriz que represente el entorno de trabajo discretizado
    # OJO!!! Cada 'tile' del piso en realidad son 4 celdas
    tam_mundo, _ = entorno.getField('floorSize').getSFVec2f()
    tam_celda, _ = entorno.getField('floorTileSize').getSFVec2f()
    num_celdas_por_lado = int((tam_mundo / tam_celda) * 2)
    grid = [[0] * num_celdas_por_lado for i in range(num_celdas_por_lado)]
    
    # Hallar ubicacion del epuck (punto inicial) y la bola (punto final)
    # OJO!!! Webots toma el eje Y saliendo del tablero, por lo tanto las coordenadas XY de la planeacion
    # equivalen a coordenadas XZ en Webots
    # OJO!!! No olvidar que estamos invirtiendo el eje de coordenadas Y
    robot_pos_x_ini, _, robot_pos_y_ini = epuck.getField('translation').getSFVec3f()
    celda_ini_x = int(math.floor((robot_pos_x_ini * num_celdas_por_lado) + (num_celdas_por_lado / 2)))
    celda_ini_y = int(math.floor((robot_pos_y_ini * num_celdas_por_lado) + (num_celdas_por_lado / 2)))
    
    bola_x, _, bola_y = bola.getField('translation').getSFVec3f()
    celda_fin_x = int(math.floor((bola_x * num_celdas_por_lado) + (num_celdas_por_lado / 2)))
    celda_fin_y = int(math.floor((bola_y * num_celdas_por_lado) + (num_celdas_por_lado / 2)))
    
    # Hallar ubicacion de los obstaculos
    obs1_x, _, obs1_y = obs1.getField('translation').getSFVec3f()
    obs2_x, _, obs2_y = obs2.getField('translation').getSFVec3f()
    obs3_x, _, obs3_y = obs3.getField('translation').getSFVec3f()
    obs4_x, _, obs4_y = obs4.getField('translation').getSFVec3f()
    
    # Hallar la mitad de la longitud de cada cara
    # Se resta el 0.01 para evitar que al usar la funcion "floor" el obstaculo ocupe mas celdas 
    # que el valor correcto
    semilado_obs, _, _ = obs1.getField('size').getSFVec3f()
    semilado_obs = (semilado_obs / 2) - 0.01
    
    # Hallar ubicacion de las 4 esquinas de los obstaculos
    esquinas1_x = [obs1_x - semilado_obs, obs1_x - semilado_obs, obs1_x + semilado_obs, 
                   obs1_x + semilado_obs]
    esquinas1_y = [obs1_y - semilado_obs, obs1_y + semilado_obs, obs1_y + semilado_obs, 
                   obs1_y - semilado_obs]
    esquinas2_x = [obs2_x - semilado_obs, obs2_x - semilado_obs, obs2_x + semilado_obs, 
                   obs2_x + semilado_obs]
    esquinas2_y = [obs2_y - semilado_obs, obs2_y + semilado_obs, obs2_y + semilado_obs, 
                   obs2_y - semilado_obs]
    esquinas3_x = [obs3_x - semilado_obs, obs3_x - semilado_obs, obs3_x + semilado_obs, 
                   obs3_x + semilado_obs]
    esquinas3_y = [obs3_y - semilado_obs, obs3_y + semilado_obs, obs3_y + semilado_obs, 
                   obs3_y - semilado_obs]
    esquinas4_x = [obs4_x - semilado_obs, obs4_x - semilado_obs, obs4_x + semilado_obs, 
                   obs4_x + semilado_obs]
    esquinas4_y = [obs4_y - semilado_obs, obs4_y + semilado_obs, obs4_y + semilado_obs, 
                   obs4_y - semilado_obs]
        
    # Guardar todas las coordenadas XY de las esquinas de los obstaculos en un unico arreglo
    celdas_obs_x = []
    celdas_obs_y = []
    for i in range(0, num_obstaculos):
      celdas_obs_x.append(esquinas1_x[i])
      celdas_obs_y.append(esquinas1_y[i])
    for i in range(0, num_obstaculos):
      celdas_obs_x.append(esquinas2_x[i])
      celdas_obs_y.append(esquinas2_y[i])
    for i in range(0, num_obstaculos):
      celdas_obs_x.append(esquinas3_x[i])
      celdas_obs_y.append(esquinas3_y[i])
    for i in range(0, num_obstaculos):
      celdas_obs_x.append(esquinas4_x[i])
      celdas_obs_y.append(esquinas4_y[i])
    
    # Con base en las esquinas, hallar celdas ocupadas por los obstaculos
    for i in range(0, len(celdas_obs_x)):
      celdas_obs_x[i] = int(math.floor((celdas_obs_x[i] * num_celdas_por_lado) + (num_celdas_por_lado / 2)))
      celdas_obs_y[i] = int(math.floor((celdas_obs_y[i] * num_celdas_por_lado) + (num_celdas_por_lado / 2)))
    
    # Almacenar los obstaculos en la matriz con un valor de -1
    for i in range(0, len(celdas_obs_x)):
      grid[celdas_obs_y[i]][celdas_obs_x[i]] = -1
      
    # Almacenar el punto inicial con un valor de 2
    grid[celda_ini_y][celda_ini_x] = 2
    grid[celda_fin_y][celda_fin_x] = 4
      
      
    # Imprimir la matriz del entorno discretizado (X son las columnas, Y son las filas)
    #print 'El espacio de trabajo discretizado es:'
    #print grid    
    
    
    # Iniciar la busqueda A*
    trayectoria = self.Astar(grid, num_celdas_por_lado, celdas_obs_x, celdas_obs_y, celda_ini_x, 
                             celda_ini_y, celda_fin_x, celda_fin_y)
    
    # Dado que el resultado de la busqueda A* va desde el punto final hacia el
    # punto inicial, invertimos la lista retornada
    trayectoria.reverse()
    
    # Descomponer la trayectoria en coordenadas X y Y
    trayectoria_x = []
    trayectoria_y = []
    for i in range(len(trayectoria)):
      if (i % 2 == 0):
        trayectoria_x.append(trayectoria[i])
      else:
        trayectoria_y.append(trayectoria[i])

    #print 'La trayectoria es: '
    #print trayectoria_x
    #print trayectoria_y
    #print ' '
    
    
    # Hallar coordenadas de los puntos intermedios de la trayectoria
    coordenadas_x = []
    coordenadas_y = []
    for i in range(len(trayectoria_x)):
      x_deseado = ((trayectoria_x[i] - float(num_celdas_por_lado / 2)) / num_celdas_por_lado) + (tam_celda / 4)
      y_deseado = ((trayectoria_y[i]- float(num_celdas_por_lado / 2)) / num_celdas_por_lado) + (tam_celda / 4)
      coordenadas_x.append(x_deseado)
      coordenadas_y.append(y_deseado)
    
    #print 'Las coordenadas globales correspondientes a dicha trayectoria son: '
    #print coordenadas_x
    #print coordenadas_y
    #print ' '
    
    
    # Hallar direcciones GLOBALES de movimiento a partir de la trayectoria
    direcciones = []
    for i in range(len(trayectoria_x) - 1):
      if (trayectoria_x[i+1] == trayectoria_x[i]) and (trayectoria_y[i+1] > trayectoria_y[i]):
        # Abajo - Orientacion 180 grados
        direcciones.append(math.pi)
      elif (trayectoria_x[i+1] == trayectoria_x[i]) and (trayectoria_y[i+1] < trayectoria_y[i]):
        # Arriba - Orientacion 0 grados
        direcciones.append(0)
      elif (trayectoria_x[i+1] > trayectoria_x[i]) and (trayectoria_y[i+1] == trayectoria_y[i]):
        # Derecha - Orientacion 270 grados
        direcciones.append(-1*(math.pi/2))
      elif (trayectoria_x[i+1] < trayectoria_x[i]) and (trayectoria_y[i+1] == trayectoria_y[i]):
        # Izquierda - Orientacion 90 grados
        direcciones.append(math.pi/2)
        
    #print 'Las orientaciones globales necesarias para la trayectoria son: '
    #print direcciones
    #print ' '
    
    
    
    # CICLO PRINCIPAL
    contador = 0
    debo_girar = False
      
    while True:
      # Perform a simulation step of 64 milliseconds
      # and leave the loop when the simulation is over
      if self.step(64) == -1:
        break  
                
      # Ejecutar la trayectoria      
      if contador < len(coordenadas_x):      
      
        # Hallar posicion y orientacion actual del robot ePuck
        robot_pos_x, _, robot_pos_y = epuck.getField('translation').getSFVec3f()
        _, _, _, robot_theta = epuck.getField('rotation').getSFRotation()    
          
             
        # Si debo ejecutar una translacion
        if (debo_girar == False):    
                        
          # Cuando la orientacion sea correcta, detener el robot - S
          if (math.sqrt(pow(robot_pos_x - coordenadas_x[contador], 2)) < 0.005) and (math.sqrt(pow(robot_pos_y - coordenadas_y[contador], 2)) < 0.005):            
            debo_girar = True
            orden = 'S' 
            continue 
            
          # Mientras las coordenadas no sean correctas, seguir avanzando - F
          else:
            orden = 'F' 
        
        
         # Si se debe ejecutar una rotacion
        elif (debo_girar == True):
        
          # Mientras la orientacion no sea correcta
          if abs(robot_theta - direcciones[contador]) > 0.02:
          
            # Hallar la direccion en la cual debemos girar
            if direcciones[contador] > robot_theta:
              # Girar en sentido antihorario - A
              orden = 'A'
            elif direcciones[contador] < robot_theta:
              # Girar en sentido horario - H
              orden = 'H'  
            
          # Cuando la orientacion sea correcta, detener el robot - S
          else:                
            debo_girar = False
            orden = 'S'   
           
            # Pasar a la siguiente pareja de instrucciones
            contador = contador + 1    
            continue 
            
            
        # Enviar el comando
        emisor.send(orden)

      
    # FIN DEL CICLO PRINCIPAL


  # Funcion que ejecuta el algoritmo de busqueda heuristica A*
  def Astar(self, grid, celdas_por_cara, celdas_obs_x, celdas_obs_y, celda_ini_x, celda_ini_y, 
            celda_fin_x, celda_fin_y):
    
    # La trayectoria comienza vacia
    trayectoria_en_celdas = []
    
    # Listas para el algoritmo
    OPEN = []
    CLOSED = []
    COMES_FROM = []
    
    # Colocar todos los obstaculos en la lista CLOSED
    for k in range(0, len(celdas_obs_x)):
      celda_ocupada = Celda(celdas_obs_x[k], celdas_obs_y[k], 1, 1, -1, -1, -1)
      CLOSED.append(celda_ocupada)
    
    # Colocar el punto de inicio en la lista OPEN
    costo_camino = 0
    distancia_hasta_meta = self.Distancia(celda_ini_x, celda_ini_y, celda_fin_x, celda_fin_y)
    nodo_inicial = Celda(celda_ini_x, celda_ini_y, celda_ini_x, celda_ini_y, costo_camino, 
                         distancia_hasta_meta, costo_camino + distancia_hasta_meta)
    OPEN.append(nodo_inicial)
    
    # INICIO DEL ALGORITMO
    print ' '
       
    # Mientras que aun queden celdas en la lista OPEN
    while len(OPEN) > 0:
      
      # Obtener el nodo actual con menor valor f(n) de la lista OPEN
      indice_min = self.Minimo_f(OPEN, len(OPEN), celda_fin_x, celda_fin_y)
      nodo_actual = OPEN[indice_min]
      
      # Si encontramos la meta, retornar la trayectoria obtenida
      if (nodo_actual.coord_x == celda_fin_x) and (nodo_actual.coord_y == celda_fin_y):
        trayectoria_en_celdas = self.Reconstruir_trayecto(grid, nodo_actual, nodo_inicial, COMES_FROM);
        return trayectoria_en_celdas
            
      # Quitar el nodo actual de la lista OPEN y colocarlo en la lista CLOSED
      del(OPEN[indice_min])
      CLOSED.append(nodo_actual)
      
      # Para todos los nodos adyacentes:
      ADYACENT = self.Hallar_vecinos(grid, nodo_actual, celda_fin_x, celda_fin_y, celdas_por_cara)         
      for i in range(0, len(ADYACENT)):
        
        # Analizar el sucesor (nodo adyacente) actual
        sucesor = ADYACENT[i]
          
        # Si el nodo adyacente actual se encuentra en la lista CLOSED, saltar una iteracion
        if self.Contiene(sucesor, CLOSED) == True:
          continue
              
        # Calcular un valor tentativo de la funcion g(n)
        gn_tentativo = nodo_actual.valor_funcion_g + self.Distancia(nodo_actual.coord_x, 
                                                                    nodo_actual.coord_y, sucesor.coord_x, 
                                                                    sucesor.coord_y)
                         
        # Si el vecino no esta en OPEN; o si gn_tentativo < g(vecino)
        if (self.Contiene(sucesor, OPEN) == False) or (gn_tentativo < sucesor.valor_funcion_g):
            
          # Actualizar los valores del vecino
          sucesor.coord_x_padre = nodo_actual.coord_x
          sucesor.coord_y_padre = nodo_actual.coord_y
          sucesor.valor_funcion_f = gn_tentativo + self.Distancia(sucesor.coord_x, sucesor.coord_y, 
                                                             celda_fin_x, celda_fin_y);
            
          # Colocarlo en la lista COMES_FROM
          COMES_FROM.append(sucesor)
            
          # Si el vecino no esta en OPEN, agregarlo a OPEN
          if (self.Contiene(sucesor, OPEN) == False):
              OPEN.append(sucesor)
        
    # Si no quedan nodos en la lista OPEN, no hay ninguna trayectoria que nos lleve del punto inicial 
    # a la meta
    if len(OPEN) < 1:
      trayectoria_en_celdas = []
        
    return trayectoria_en_celdas
  
  
  
  # Funcion para calcular la distancia entre dos celdas
  def Distancia(self, x1, y1, x2, y2):
    return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))
    
    
    
  # Verificar si un nodo esta dentro de una lista
  def Contiene(self, nodo, LISTA):
    nodo_ya_existe = False
    for i in range(0, len(LISTA)):
      if (nodo.coord_x == LISTA[i].coord_x) and (nodo.coord_y == LISTA[i].coord_y):
        nodo_ya_existe = True
        break
    return nodo_ya_existe



  # Funcion para retornar el indice del nodo con menor valor de f(n)
  def Minimo_f(self, LISTA, tam_LISTA, celda_fin_x, celda_fin_y):
    valores_f = []
    bandera_meta = False
    indice_min = -1
    
    # Revisar todos los nodos actualmente dentro de la lista OPEN
    for j in range(0, tam_LISTA):
      # Agregar el valor de f(n) del nodo actual al arreglo
      valores_f.append(LISTA[j].valor_funcion_f)
        
      # Si encontramos el nodo objetivo
      if (LISTA[j].coord_x == celda_fin_x) and (LISTA[j].coord_y == celda_fin_y):
        # Guardar el indice del nodo objetivo y activar la bandera
        bandera_meta = True
        indice_min = j   
        break
    
    # De lo contrario, si el arreglo temporal no tiene un tamano nulo
    if (bandera_meta == False) and (len(valores_f) > 0):   
      # Calcular el indice del nodo con menor valor en el arreglo temporal
      f_min = min(valores_f)
      indice_min = valores_f.index(f_min)
        
    # Si el arreglo temporal es de tamano nulo (esta vacio)
    elif len(valores_f) < 1:
      # No hay mas caminos disponibles a evaluar para la busqueda
      indice_min = -1
        
    return indice_min
    
    
    
  # Funcion para expandir un nodo y retornar la lista de sucesores con sus
  # correspondientes valores calculados de f(n)
  def Hallar_vecinos(self, grid, nodo, celda_fin_x, celda_fin_y, celdas_por_cara):
    vecinos = []
    coordenadas_vecinos = [[-1, 0], [1, 0], [0, -1], [0, 1]]
    
    # Evaluar las 8 celdas que rodean al nodo actual
    for i in range(0, len(coordenadas_vecinos)):
      sucesor_x = nodo.coord_x + coordenadas_vecinos[i][0]
      sucesor_y = nodo.coord_y + coordenadas_vecinos[i][1]
    
      # Mientras que el nodo este dentro de los limites del arreglo
      if (sucesor_x >= 0 and sucesor_x < celdas_por_cara) and (sucesor_y >= 0 and sucesor_y < celdas_por_cara):
    
        # Y el vecino NO sea un obstaculo
        if (grid[sucesor_x][sucesor_y] != -1):
    
          # Costo de viajar al nodo
          gn = nodo.valor_funcion_h + self.Distancia(nodo.coord_x, nodo.coord_y, sucesor_x, sucesor_y)
          # Distancia estimada entre el nodo y la meta
          hn = self.Distancia(sucesor_x, sucesor_y, celda_fin_x, celda_fin_y)
          # Calculo de f(n)
          fn = gn + hn
    
          # Crear el sucesor y agregarlo a la lista de adyacentes
          sucesor = Celda(sucesor_x, sucesor_y, nodo.coord_x, nodo.coord_y, gn, hn, fn)       
          vecinos.append(sucesor)
    
    return vecinos
    
  
  
  # Funcion para reconstruir la trayectoria
  def Reconstruir_trayecto(self, grid, nodo_actual, nodo_inicial, COMES_FROM):

    # La trayectoria inicia vacia
    trayectoria = [];
    
    # Agregar nodo actual (al comenzar, el nodo de la meta) a la trayectoria
    #trayectoria.append('[' + str(int(nodo_actual.coord_x)) + ',' + str(int(nodo_actual.coord_y)) + '] ')
    trayectoria.append(int(nodo_actual.coord_y))
    trayectoria.append(int(nodo_actual.coord_x))
    
    # Mientras que el nodo actual este dentro de la lista COMES_FROM
    while (self.Contiene(nodo_actual, COMES_FROM)):
    
      # El nuevo nodo actual sera el padre del nodo actual
      for i in range (0, len(COMES_FROM)):
        if (COMES_FROM[i].coord_x == nodo_actual.coord_x_padre) and (COMES_FROM[i].coord_y == nodo_actual.coord_y_padre):
          nodo_actual = COMES_FROM[i];
          break
                
      # Agregar nodo actual a la trayectoria
      #trayectoria.append('[' + str(int(nodo_actual.coord_x)) + ',' + str(int(nodo_actual.coord_y)) + '] ')
      trayectoria.append(int(nodo_actual.coord_y))
      trayectoria.append(int(nodo_actual.coord_x))
      
        
      # Si el padre es el punto inicial, lo agregamos y terminamos la
      # reconstruccion
      if (nodo_actual.coord_x_padre == nodo_inicial.coord_x) and (nodo_actual.coord_y_padre == nodo_inicial.coord_y):
        #trayectoria.append('[' + str(int(nodo_actual.coord_x)) + ',' + str(int(nodo_actual.coord_y)) + '] ')
        break
    
    return trayectoria
    
    
    
# This is the main program of your controller.
# It creates an instance of your Robot subclass, launches its
# function(s) and destroys it at the end of the execution.
# Note that only one instance of Robot should be created in
# a controller program.
controller = Busqueda_astar()
controller.Ejecutar()
