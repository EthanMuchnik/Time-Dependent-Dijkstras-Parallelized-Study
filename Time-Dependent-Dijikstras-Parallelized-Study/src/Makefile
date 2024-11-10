# APP_NAME = tdd_seq
# OBJS = tdd_seq.o

# APP_NAME = tdd_par_relax
# OBJS = tdd_par_relax.o

APP_NAME = tdd_par
OBJS = tdd_par.o

CXX = g++
CXXFLAGS = -Wall -O3 -std=c++17 -m64 -I. -fopenmp -Wno-unknown-pragmas


# Rule to link the application
$(APP_NAME): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(APP_NAME) $(OBJS)

# Rule to compile the source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule to remove object files and the application binary
clean:
	rm -f $(OBJS) $(APP_NAME)