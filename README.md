# P3
Repository for the Artificial Intelligence signature of the UCM. Practice 3
___________________________________________________________________________

<!---
 # Video de pruebas [aqu�](https://youtu.be/ztK_XIu5nJk)
-->

# Autores

## Grupo 02
L�pez Ben�tez, �ngel   -   angelo06@ucm.es <br>
Rave Robayo, Jose Daniel   -   jrave@ucm.es <br>
Prado Echegaray, Iv�n   -   ivprado@ucm.es <br>
Mendoza Reyes, Juan Diego   -   juandiem@ucm.es <br>


## Resumen

La pr�ctica consiste en implementar un prototipo de una simulaci�n de la novela **El fantasma de la opera**, 
La novela cuenta la historia de Erik, un m�sico deforme que vive escondido en el s�tano del Palacio de Garnier,
despu�s de componer su gran obra, Erik sue�a con que sea interpretada por Christine Daa�, y tiene la intenci�n de secuestrarla con el objetivo
de que esta memorice la obra, pero debe espantar al p�blico de la �pera y est� el vizconde Raoul de Chagny para comprometer sus planes. <br>

<br>
La pr�ctica consta de una serie de estancias en la que se movera el Fantasma (controlado por **IA**) y el vizconde(controlado por el jugador).<br>
El fantasma intentar� secuestras a la cantante para llevarla a su celda, mientras que la cantante actua en el escenario y va a la bambalinas entre actuaciones. <br>
El minotauro empieza en el centro del laberinto y merodea por los caminos del mismo hasta que ve a Teseo, en este momento empieza a **perseguirlo**, hasta dar con �l. <br>
Mientras tanto el jugador puede poner remedio a los actos del fantasma, recolocar las l�mparas que tire, sacar a la cantante de la celda y ensa�arse con los muebles
de la guarida del fantasma, lo que hace que este se enfade, si el jugador no interviene, el fantasma rapta a la cantante<br>

## Estancias

+ **Patio de butacas(P).** Estancia incial del p�blico, dividido en Este y Oeste. Los espectadores huyen al vest�bulo si cae la l�mpara correspondiente a su lado del patio de butacas,
Esta conectada con el escenario y el vest�bulo, y es visible desde los palcos.

+ **Vest�bulo(V).** Es la zona m�s externa de la �pera, donde van los bloques de p�blico
cuando se asustan. Simplemente conecta con el patio de butacas.

+ **Escenario (E).** Estancia inicial de la cantante, que intercala con las bambalinas(donde se toma su descanso), tambi�n conecta con el patio de butacas, los palcos y es posible dejarse caer
al s�tano oeste, aunque no es posible volver

Las funcionalidades minimas que se piden son: 

+ **A.** Mostrar el entorno virtual (el laberinto del Minotauro) de tama�o configurable, con un esquema de divisi�n de grafo de baldosas que incluir� una baldosa de salida, donde se ubica inicialmente el avatar (Teseo). Debe haber varios caminos alternativos para llegar a la salida, algunos m�s anchos y otros muy estrechos (pasillos de una �nica baldosa de anchura). El avatar estar� controlado por el jugador mediante los cursores [0,5 ptos.]. <br><br>
+ **B.** Situar en el centro del laberinto al agente inteligente que representa al enemigo (el Minotauro), quien realizar� un merodeo constante, pasando a perseguir al 	avatar si se lo encuentra en su l�nea de visi�n [0,5 ptos.].<br><br>
+ **C.** Representar el hilo de Ariadna (camino m�s corto a la baldosa de salida) pintado con una l�nea blanca y destacando las baldosas tambi�n con c�rculos blancos, a la vez que se activa la navegaci�n autom�tica de Teseo hasta la salida, todo ello mientras se mantenga
pulsada la barra espaciadora [1 pto.].<br><br>
+ **D.** Elegir (activando o desactivando la funcionalidad con la tecla S) si suavizar o no el camino generado por el algoritmo anterior, generalmente reduciendo las baldosas que forman parte del camino suavizado a la salida [1 pto.].<br><br>
+ **E.** Desarrollar el movimiento completo de Teseo, que mientras tenemos pulsada la barra espaciadora, va movi�ndose autom�ticamente siguiendo el hilo hacia la baldosa de salida. Esto hace desaparecer la parte del hilo que ya se ha recorrido, hilo que desaparece al completo en cuanto se suelta la barra espaciadora y se vuelve al movimiento manual [1 pto.].<br><br>



Modelo de Minotauro             |  Modelo de Teseo
:-------------------------:|:-------------------------:
<img src="https://github.com/IAV22-G02/P2/blob/main/Minotauro.png" alt="drawing" width="200"/>  |  <img src="https://github.com/IAV22-G02/P2/blob/main/Teseo.png" alt="drawing" width="200"/>

# Descripci�n Punto de Partida

## [Commit](https://github.com/IAV22-G02/P2/commit/ed26cda0429e6a28e262607879a93b947d5fc54e) de Punto de Partida 

La escena incial contiene un **Grid** con un mapa de prueba (�ste se puede generar **proceduralmente**), modelos para el minotauro y teseo, materiales y prefabs para los obstaculos adem�s de los siguientes scripts:

**BinaryHip**: Una pila de informaci�n �til para ordenar datos e implementar colas de prioridad.<br>
**Edge**: Conexi�n entre nodos, �til para calcular el coste de atravesar una casilla.<br>
**Graph**: Clase abstracta para implementar grafos.<br>
**GraphGrid**: Clase que genera el mapa a partir de un archivo .map.<br>
**TesterGrahh**: Clase que contiene distintos algoritmos para encontrar caminos en grafos. <br>
**Vertex**: Cada uno de los v�rtices del grafo. <br>

# Estructura de Clases
![text](https://github.com/IAV22-G02/P2/blob/main/UMLMaze.png)

<br>

## Descripci�n de la Soluci�n

La soluci�n consta de la implementaci�n de 3 nuevos componentes, cuyo pseudoc�digo est� m�s abajo:
+ Componente MapGenerator, que se encarga de la generaci�n prodecdural de laberintos.
+ El componente PathFinder, que usaremos para encontrar el camino m�s corto mediante el hilo de Ariadna.
+ El componente FollowPath, que llevar� a Teseo a seguir el camino que le indique el hilo de Ariadna. El pseudoc�digo de �ste componente no est� ya que solo consiste en seguir una secuencia de posiciones.

Adem�s usaremos los componente implmentados en la pr�ctica 1, que se puden ver en este enlace()
Adem�s vamos reutilizar gran parte de la estructura de movimiento de la Practica 1. Sobretodo los componentes para el movimiento de Teseo. Por otro lado, cambiaremos un poco el comportamiento de Merodeo del Minotauro para que se mueva como si estuviera apatrullando el mapa. Mas abajo se puede ver el c�digo de lo que se tiene en mente. La explicacion de la estructura se puede ver [aqu�](https://github.com/IAV22-G02/P1)

### Opcionales

La soluci�n tambi�n consta de funcionalidades opcionlaes tales como:
+ Generaci�n procedimental de laberintos (realizado)
+ Permite a�adir m�s salidas al laberinto y modifica a Teseo para que, si hay varias
salidas, salga por la m�s cercana, utilizando para ello el algoritmo de Dijkstra.


El pseudoc�digo de dichos componentes:

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

		 # If we didn�t find a shorter route, skip.
		 if endNodeRecord.costSoFar <= endNodeCost:
		 	continue

			 # Otherwise remove it from the closed list.
			 closed -= endNodeRecord

			 # We can use the node�s old cost values to calculate
			 # its heuristic without calling the possibly expensive
			 # heuristic function.
			 endNodeHeuristic = endNodeRecord.estimatedTotalCost -
			 endNodeRecord.costSoFar

			 # Skip if the node is open and we�ve not found a better
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

			  # Otherwise we know we�ve got an unvisited node, so make a
			  # record for it.
		  else:
			  endNodeRecord = new NodeRecord()
			  endNodeRecord.node = endNode

			  # We�ll need to calculate the heuristic value using
			  # the function, since we don�t have an existing record
			  # to use.
			  endNodeHeuristic = heuristic.estimate(endNode)

			  # We�re here if we need to update the node. Update the
			  # cost, estimate and connection.
			  endNodeRecord.cost = endNodeCost
			  endNodeRecord.connection = connection
			  endNodeRecord.estimatedTotalCost = endNodeCost +
			 endNodeHeuristic

		 # And add it to the open list.
		 if not open.contains(endNode):
			 open += endNodeRecord

		 # We�ve finished looking at the connections for the current
		 # node, so add it to the closed list and remove it from the
		 # open list.
		 open -= current
		 closed += current

	 # We�re here if we�ve either found the goal, or if we�ve no more
	 # nodes to search, find which.
	 if current.node != goal:
	 # We�ve run out of nodes without finding the goal, so there�s
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
  # The maximum rotation speed we�d like, probably should be smaller
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