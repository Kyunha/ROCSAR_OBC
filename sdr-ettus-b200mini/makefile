CXX      = g++
CXXFLAGS = -std=c++17 -O2 -pthread -MMD -MP \
           -I/usr/include \
           -I$(CURDIR)

LDFLAGS  = -L/usr/lib/aarch64-linux-gnu -luhd -pthread

TARGET   = connect
SRCS     = connect.cpp \
           libs/signal/signal.cpp \
           libs/tx/tx.cpp \
           libs/rx/rx.cpp

OBJS     = $(SRCS:.cpp=.o)
DEPS     = $(SRCS:.cpp=.d)

.PHONY: all clean

all: $(TARGET)

# Link object files into final executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Compile individual source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Include generated header dependency files
-include $(DEPS)

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS)