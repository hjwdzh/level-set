CC=cl
CFLAGS=-c -g -Wall
LDFLAGS= -lglu32 -lglut32 -lopengl32
SOURCES=Bound.cpp Bounds.cpp Camera.cpp control.cpp display.cpp ForceField.cpp Geometric.cpp Geometrics.cpp main.cpp Particle.cpp Plane.cpp Rigid_Geometry.cpp scene.cpp Solver.cpp Spring.cpp Springs.cpp SysDynPtc.cpp SystemPhy.cpp vmath.cpp BOUNDARY.cpp FLOOD_FILL.cpp GRID.cpp LEVELSET_MAKER.cpp LEVELSET.cpp RANGE.cpp TRIANGLE.cpp TRIANGULATED_SURFACE.cpp

OBJECTS=$(SOURCES:.cpp=.obj)
EXECUTABLE=level

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.obj:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm *.obj
	rm level
