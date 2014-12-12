CC=c++
CFLAGS=-I SimLib -I System -I Geometric -I Engine -I Tools -I User -I Bound -I ./ -c -Wall
LDFLAGS= -framework OpenGL -framework GLUT
SOURCES= main.cpp \
SimLib/BOUNDARY.cpp \
SimLib/FLOOD_FILL.cpp \
SimLib/GRID.cpp \
SimLib/GRID.cpp \
SimLib/LEVELSET_MAKER.cpp \
SimLib/LEVELSET.cpp \
SimLib/RANGE.cpp \
SimLib/TRIANGLE.cpp \
SimLib/TRIANGULATED_SURFACE.cpp \
System/SysBowling.cpp \
System/SysDynPtc.cpp \
System/SystemPhy.cpp \
Geometric/Geometric.cpp \
Geometric/Geometrics.cpp \
Geometric/Particle.cpp \
Geometric/Rigid_Geometry.cpp \
Engine/ForceField.cpp \
Engine/Solver.cpp \
Tools/Camera.cpp \
Tools/texture.cpp \
Tools/TriManager.cpp \
Tools/vmath.cpp \
User/control.cpp \
User/display.cpp \
User/scene.cpp \
Bound/Bound.cpp \
Bound/Bounds.cpp \
Bound/Plane.cpp \
Bound/Spring.cpp \
Bound/Springs.cpp

OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=level

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm SimLib/*.o
	rm System/*.o
	rm Geometric/*.o
	rm Engine/*.o
	rm Tools/*.o
	rm User/*.o
	rm Bound/*.o
	rm *.o
	rm level
