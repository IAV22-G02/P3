# P3
Repository for the Artificial Intelligence signature of the UCM. Practice 3
___________________________________________________________________________

<!---
 # Video de pruebas [aquí](https://youtu.be/ztK_XIu5nJk)
-->

# Autores

## Grupo 02
López Benítez, Ángel   -   angelo06@ucm.es <br>
Rave Robayo, Jose Daniel   -   jrave@ucm.es <br>
Prado Echegaray, Iván   -   ivprado@ucm.es <br>
Mendoza Reyes, Juan Diego   -   juandiem@ucm.es <br>


## Resumen

La práctica consiste en implementar un prototipo de una simulación de la novela **El fantasma de la opera**, 
La novela cuenta la historia de Erik, un músico deforme que vive escondido en el sótano del Palacio de Garnier,
después de componer su gran obra, Erik sueña con que sea interpretada por Christine Daaé, y tiene la intención de secuestrarla con el objetivo
de que esta memorice la obra, pero debe espantar al público de la ópera y está el vizconde Raoul de Chagny para comprometer sus planes. <br>

<br>
La práctica consta de una serie de estancias en la que se movera el Fantasma (controlado por **IA**) y el vizconde(controlado por el jugador).<br>
El fantasma intentará secuestras a la cantante para llevarla a su celda, mientras que la cantante actua en el escenario y va a la bambalinas entre actuaciones. <br>
El minotauro empieza en el centro del laberinto y merodea por los caminos del mismo hasta que ve a Teseo, en este momento empieza a **perseguirlo**, hasta dar con él. <br>
Mientras tanto el jugador puede poner remedio a los actos del fantasma, recolocar las lámparas que tire, sacar a la cantante de la celda y ensañarse con los muebles
de la guarida del fantasma, lo que hace que este se enfade, si el jugador no interviene, el fantasma rapta a la cantante<br>

## Estancias y comportamiento en estas

+ **Patio de butacas(P).** Estancia incial del público, dividido en Este y Oeste. Los espectadores huyen al vestíbulo si cae la lámpara correspondiente a su lado del patio de butacas,
Esta conectada con el escenario y el vestíbulo, y es visible desde los palcos.

+ **Vestíbulo(V).** Es la zona más externa de la ópera, donde van los bloques de público
cuando se asustan. Simplemente conecta con el patio de butacas.

+ **Escenario(E).** Estancia inicial de la cantante, que intercala con las bambalinas(donde se toma su descanso), también conecta con el patio de butacas, los palcos y es posible dejarse caer
al sótano oeste, aunque no es posible volver, el fantasma no puede pisar estas estancias si hay público mirando, aunque puede capturar a la cantante y llevársela a donde quiera, soltándola 
por voluntad propia o porque se sienta intimidado por la presencia del vizconde, si la cantante acaba en una estancia desconocida, empieza a vagar hasta ser encontrada por el vizconde
que la lleva hasta una estancia que ella conozca, donde pueda volver al escenario o a las bambalinas

+ **Bambalinas(B).** Estancia donde suele descansar la cantante y que conecta con el escenario, el sótano oeste y que permite deslizarse por una rampa algo oculta al sótano este, sin posibilidad de regresar. 

+ **Palco oeste(Po).** Estancia inicial del vizconde, tiene una palanca para dejar caer la lámpara oeste del patio de butacas. 
Conecta con el escenario, con el sótano oeste y permite ver el patio de butacas 

+ **Palco oeste(Pe).** Estancia similar al palco oeste, con una palanca que se puede usar para dejar caer la lámpara este del patio de butacas. 
Conecta con el escenario, con el sótano este y permite ver el patio de butacas, aunque sin visibilidad en el otro sentido.

+ **Sótano oeste(So).**  Estancia que conecta con el palco oeste, con las bambalinas y con el
sótano norte, aunque para recorrer esta conexión hace falta subirse a una barca. Solo puede subirse una persona(o una con una en brazos). La barca comienza en el sótano norte,
pero hay una palanca para acercarla a cualquier lado del sótano.

+ **Sótano este(Se).**  Estancia que conecta con el palco este, y tanto con el sótano norte
como con la sala de música donde compone su obra el fantasma, aunque para recorrer estas dos
últimas conexiones hacen falta barcas. Por defecto, la barca que lleva al sótano norte sí está en
esta orilla, pero la que lleva a la sala de música está en la orilla contraria. Aunque se puede
llegar a esta estancia desde las bambalinas, por una trampilla, desde aquí no se conecta con las
bambalinas.

+ **Celda(C).**  Estancia donde el fantasma deja a la cantante para completar su secuestro
con éxito, usando una palanca que activa unas rejas que la impiden salir (y que por supuesto el
vizconde podrá desactivar). Conecta con el sótano norte.

+ **Sótano norte(N).** Estancia que conecta con la celda, además de con la sala de música,
el sótano este y el sótano oeste a través de sus correspondientes tres barcas.

+ **Sala de música(M)** Estancia inicial del fantasma, conecta mediante una barca con el sótano este, y con otra con el sótano
norte.. El fantasma tiene el objetivo principal de secuestrar a la cantante, para lo que intentará
buscarla en las bambalinas, en el escenario o si no logra dar con ella, explorando las demás
estancias meticulosamente por si estuviera “perdida” por allí. No puede acceder al escenario si
hay público mirando, de modo que, como objetivo secundario, necesita tirar las dos lámparas
del techo para vaciar del todo el patio de butacas. Sea como sea, una vez atrapada la cantante, la
llevará consigo hasta la celda, intentando usar siempre el camino con menor coste (recordando
la última posición de las barcas y del vizconde que conoce, y eligiendo la ruta con menor coste,
la que tenga más barcas a su favor y que evite al héroe de esta historia). Cuando llega hasta la
celda la soltará allí, activará las rejas e irá hasta la sala de música, permaneciendo allí
indefinidamente. Lo único que desconcentra al fantasma cuando está componiendo es escuchar
a su musa cantar de nuevo en el escenario, reavivando sus deseos de secuestrarla y encerrarla
otra vez en su celda. Por otro lado, si el fantasma llega a percibir el ruido de los golpes del
vizconde a su piano, abandonará lo que esté haciendo (soltando a la cantante) y correrá
enfurecido hasta allí para dedicar unos segundos a arreglar semejante estropicio.



Las funcionalidades minimas que se piden son: 

+ **A.** Mostrar el entorno virtual (la casa de la ópera), con un esquema de división de malla de
navegación proporcionado por Unity, donde se ubiquen todos los elementos descritos
anteriormente. El vizconde será controlado libremente por el jugador mediante los
cursores y una única tecla de acción para interactuar con otros elementos. Aunque haya
cámaras que sigan a cada uno de los personajes, conviene que haya una adicional que
nos dé la vista general del entorno [0,5 ptos.].<br><br>

+ **B.** Hacer que parte del público huya tras la caída de una lámpara, y regrese en cuanto esté
arreglada. Será una navegación y un movimiento trivial, sin apenas decisión [0,5 ptos.].<br><br>

+ **C.** Representar a la cantante como un agente inteligente basado en una máquina de estados
que pasa del escenario a las bambalinas cuando toca, que puede ser “llevada” por los
otros dos personajes hasta otra estancia, que navega algo desorientada cuando está en
las estancias subterráneas, y que se deja llevar por el vizconde, con la esperanza de
reencontrar el escenario y continuar su rutina allí. Tiene navegación, movimiento y
percepción sencillos, y decisión mediante máquina de estados [1 ptos.].<br><br>

+ **D.** Desarrollar el árbol de comportamiento completo del fantasma, para que busque a la
cantante, tire las lámparas, la capture, la lleve a la celda, active las rejas, etc. [1 pto.]<br><br>

+ **E.** Usar un sistema de gestión sensorial para que el fantasma reaccione realmente a lo que
ve (en la propia estancia o estancias vecinas visibles) y lo que oye (el canto de su musa
y el ruido de la sala de música), sin tener que recurrir a información privilegiada
(únicamente recordando lo que ha visto anteriormente) [1 pto.]. <br><br>



Modelo de Minotauro             |  Modelo de Teseo
:-------------------------:|:-------------------------:
<img src="https://github.com/IAV22-G02/P2/blob/main/Minotauro.png" alt="drawing" width="200"/>  |  <img src="https://github.com/IAV22-G02/P2/blob/main/Teseo.png" alt="drawing" width="200"/>

# Descripción Punto de Partida

## [Commit](https://github.com/IAV22-G02/P2/commit/ed26cda0429e6a28e262607879a93b947d5fc54e) de Punto de Partida 

La escena incial contiene un **Grid** con un mapa de prueba (éste se puede generar **proceduralmente**), modelos para el minotauro y teseo, materiales y prefabs para los obstaculos además de los siguientes scripts:

**BinaryHip**: Una pila de información útil para ordenar datos e implementar colas de prioridad.<br>
**Edge**: Conexión entre nodos, útil para calcular el coste de atravesar una casilla.<br>
**Graph**: Clase abstracta para implementar grafos.<br>
**GraphGrid**: Clase que genera el mapa a partir de un archivo .map.<br>
**TesterGrahh**: Clase que contiene distintos algoritmos para encontrar caminos en grafos. <br>
**Vertex**: Cada uno de los vértices del grafo. <br>

# Estructura de Clases
![text](https://github.com/IAV22-G02/P2/blob/main/UMLMaze.png)

<br>

## Descripción de la Solución

La solución consta de la implementación de 3 nuevos componentes, cuyo pseudocódigo está más abajo:
+ Componente MapGenerator, que se encarga de la generación prodecdural de laberintos.
+ El componente PathFinder, que usaremos para encontrar el camino más corto mediante el hilo de Ariadna.
+ El componente FollowPath, que llevará a Teseo a seguir el camino que le indique el hilo de Ariadna. El pseudocódigo de éste componente no está ya que solo consiste en seguir una secuencia de posiciones.

Además usaremos los componente implmentados en la práctica 1, que se puden ver en este enlace()
Además vamos reutilizar gran parte de la estructura de movimiento de la Practica 1. Sobretodo los componentes para el movimiento de Teseo. Por otro lado, cambiaremos un poco el comportamiento de Merodeo del Minotauro para que se mueva como si estuviera apatrullando el mapa. Mas abajo se puede ver el código de lo que se tiene en mente. La explicacion de la estructura se puede ver [aquí](https://github.com/IAV22-G02/P1)

### Opcionales

La solución también consta de funcionalidades opcionlaes tales como:
+ Generación procedimental de laberintos (realizado)
+ Permite añadir más salidas al laberinto y modifica a Teseo para que, si hay varias
salidas, salga por la más cercana, utilizando para ello el algoritmo de Dijkstra.


El pseudocódigo de dichos componentes:

## MazeGenerator (Map)
```python
function maze(level: Level, start: Location):
 # A stack of locations we can branch from. locations = [start]
 level.startAt(start)

 while locations:
 current = locations.top()

 # Try to connect to a neighboring location.
 next = level.makeConnection(current)
 if next:
 # If successful, it will be our next iteration.
 locations.push(next)
 else:
 locations.pop()

class Level:
	function startAt(location: Location)
	function makeConnection(location: Location) -> Location

class Location:
	x: int
	x: int
	x: int
	y: int

 class Connections:
	inMaze: bool = false
	directions: bool[4] = [false, false, false, false]

 class GridLevel:
	# dx, dy, and index into the Connections.directions array.
	NEIGHBORS = [(1, 0, 0), (0, 1, 1), (0, -1, 2), (-1, 0, 3)]

	width: int
	height: int
	cells: Connections[width][height]

	function startAt(location: Location):
	cells[location.x][location.y].inMaze = true

 function canPlaceCorridor(x: int, y: int, dirn :int) -> bool:
	# Must be in-bounds and not already part of the maze.
	return 0 <= x < width and
	0 <= y < height and
	not cells[x][y].inMaze

 function makeConnection(location: Location) -> Location:
	# Consider neighbors in a random order.
	neighbors = shuffle(NEIGHBORS)

	x = location.x
	y = location.y
	for (dx, dy, dirn) in neighbors:

		# Check if that location is valid.
		nx = x + dx
		ny = y + dy
		fromDirn = 3 - dirn
		if canPlaceCorridor(nx, ny, fromDirn):

		#Perform the connection.
		cells[x][y].directions[dirn] = true
		cells[nx][ny].inMaze = true
		cells[nx][ny].directions[fromDirn] = true return Location(nx, ny)

 # null of the neighbors were valid.
 return null
```

## PathFinder(Teseo)
```python

 function pathfindAStar(graph: Graph,
	 start: Node,
	 end: Node,
	 heuristic: Heuristic
	 ) -> Connection[]:
	 # This structure is used to keep track of the
	 # information we need for each node.
	 class NodeRecord:
	 node: Node
	 connection: Connection
	 costSoFar: float
	 estimatedTotalCost: float

	 # Initialize the record for the start node.
	 startRecord = new NodeRecord()
	 startRecord.node = start
	 startRecord.connection = null
	 startRecord.costSoFar = 0
	 startRecord.estimatedTotalCost = heuristic.estimate(start)

	 # Initialize the open and closed lists.
	 open = new PathfindingList()

	 open += startRecord
	 closed = new PathfindingList()

	 # Iterate through processing each node.
	 while length(open) > 0:
		 # Find the smallest element in the open list (using the
		 # estimatedTotalCost).
		 current = open.smallestElement()

		 # If it is the goal node, then terminate.
		 if current.node == goal:
		 	break

		 # Otherwise get its outgoing connections.
		 connections = graph.getConnections(current)

		 # Loop through each connection in turn.
		 for connection in connections:
			 # Get the cost estimate for the end node.
			 endNode = connection.getToNode()
			 endNodeCost = current.costSoFar + connection.getCost()

		 # If the node is closed we may have to skip, or remove it
		 # from the closed list.
		 if closed.contains(endNode):
			 # Here we find the record in the closed list
			 # corresponding to the endNode.
			 endNodeRecord = closed.find(endNode)

		 # If we didn’t find a shorter route, skip.
		 if endNodeRecord.costSoFar <= endNodeCost:
		 	continue

			 # Otherwise remove it from the closed list.
			 closed -= endNodeRecord

			 # We can use the node’s old cost values to calculate
			 # its heuristic without calling the possibly expensive
			 # heuristic function.
			 endNodeHeuristic = endNodeRecord.estimatedTotalCost -
			 endNodeRecord.costSoFar

			 # Skip if the node is open and we’ve not found a better
			 # route.
		 else if open.contains(endNode):
			 # Here we find the record in the open list
			 # corresponding to the endNode.

		 	endNodeRecord = open.find(endNode)

		  # If our route is no better, then skip.
		  if endNodeRecord.costSoFar <= endNodeCost:
		  	continue

			  # Again, we can calculate its heuristic.
			  endNodeHeuristic = endNodeRecord.cost -
			  endNodeRecord.costSoFar

			  # Otherwise we know we’ve got an unvisited node, so make a
			  # record for it.
		  else:
			  endNodeRecord = new NodeRecord()
			  endNodeRecord.node = endNode

			  # We’ll need to calculate the heuristic value using
			  # the function, since we don’t have an existing record
			  # to use.
			  endNodeHeuristic = heuristic.estimate(endNode)

			  # We’re here if we need to update the node. Update the
			  # cost, estimate and connection.
			  endNodeRecord.cost = endNodeCost
			  endNodeRecord.connection = connection
			  endNodeRecord.estimatedTotalCost = endNodeCost +
			 endNodeHeuristic

		 # And add it to the open list.
		 if not open.contains(endNode):
			 open += endNodeRecord

		 # We’ve finished looking at the connections for the current
		 # node, so add it to the closed list and remove it from the
		 # open list.
		 open -= current
		 closed += current

	 # We’re here if we’ve either found the goal, or if we’ve no more
	 # nodes to search, find which.
	 if current.node != goal:
	 # We’ve run out of nodes without finding the goal, so there’s
	 # no solution.
	 return null

	 else:
	 # Compile the list of connections in the path.

	 path = []

	 # Work back along the path, accumulating connections.
	 while current.node != start:
	 path += current.connection
	 current = current.connection.getFromNode()

	 # Reverse the path, and return it.
	 return reverse(path)


```

### Wander 2 (Ours) (Minotauro)
```python
  # Is intersection uses navigation instead of movement
  function IsIntersection(position)-> bool:
  	coord : vector2
	coord = posToGrid(position)
	
	return haveNeighboursIntersection(coord.x, coord.y)
  
  
class KinematicWander :
  character: Static
  maxSpeed: float
  position: vector2
  # The maximum rotation speed we’d like, probably should be smaller
  # than the maximum possible, for a leisurely change in direction.
  maxRotation: float
 
  function getSteering() -> KinematicSteeringOutput:
	result = new KinematicSteeringOutput()

	newDir: vector2 
	if(isIntersection(position) || randomBinomial(0, 10) < 3)
		toDir = randomDirection(position)

		newDir = toDir - position; 

		# Get velocity from the vector form of the orientation.
		result.velocity = maxSpeed * newDir

	return newDir;
```

## Seek(Minotauro)
```python
class KinematicSeek:
 character: Static
 target: Static

 maxSpeed: float

 function getSteering() -> KinematicSteeringOutput:
	 result = new KinematicSteeringOutput()
	 # Get the direction to the target.
	 result.velocity = theseus.position - character.position

	 # The velocity is along this direction, at full speed.
	 result.velocity.normalize()
	 result.velocity *= maxSpeed

	 result.rotation = 0
	 return result
```


# Referencias Usadas:
+ AI for GAMES Third Edition, Ian Millintong
+ Unity 5.x Game AI Programming Cookbook, Jorge Palacios