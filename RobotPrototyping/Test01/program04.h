#pragma once

class Program04 {
public:
	typedef std::pair<double, double> Range;
	class Motor;

	Timer                  timer;
	Physics                physics;
	GLViewWindow           window;
	PickObjectController   pickCtrl;
	bool                   pauseSimulation = true;
	std::shared_ptr<Motor> motor1, motor2;

	class Graph
	{
	public:
		typedef std::string               ChannelName;

		class Channel {
		public:
			typedef boost::circular_buffer<double> CircularBuffer;
			struct Point { double x, y; };
			typedef std::vector<Point> Points;
			struct Color { double r, g, b; };
			typedef std::vector<Color> ColorPalette;

			Graph*               graph = nullptr; // parent graph
			const int            MAX_POINTS = 100;
			CircularBuffer       buffer;
			double               lastPointTime = 0;
			Color                color = { 0, 0, 0 };
			boost::optional<Range> fixedRange = boost::none;

			Channel(Graph *graph) : 
				buffer(MAX_POINTS), 
				graph(graph)
			{
				color = newColor();
			}
			
			static ColorPalette& colorPalette() {
				static ColorPalette pal = {
					{ 1, 0, 0 },
					{ 0, 1, 0 },
					{ 0, 0, 1 },
					{ 1, 1, 0 },
					{ 0, 1, 1 },
					{ 1, 0, 1 },
				};
				return pal;
			}

			static Color newColor() {
				static int i = 0;
				auto& pal = colorPalette();
				return pal[i++ % pal.size()];
			}

			void addPoint(double value) {
				auto currentTime = graph->currentTime;
				auto preci = precision();

				// First point into buffer.
				if (buffer.empty()) {
					lastPointTime = currentTime;
					buffer.push_back(value);
					return;
				}

				// What is the distance horizontally between new point
				// and the last point in memory?
				auto durationFromLast = currentTime - lastPointTime;

				// Add point to buffer only if time difference between
				// last data point and current data point greater than 
				// precision.
				if (durationFromLast < preci) {
					// Overwrite last value in buffer, since new datapoint
					// is too close.
					buffer.back() = value;
				}
				else if (durationFromLast < preci * 2) {
					// New data point is directly next to last point.
					// Simply append buffer.
					buffer.push_back(value);
					lastPointTime += preci;
				}
				else {
					// New data point is much too far ahead. We have 
					// to add interpolated points in between.
					int numPoints = (int)std::floor(durationFromLast / preci);
					auto val1 = buffer.back();
					auto val2 = value;
					for (int i = 1; i <= numPoints; i++) {
						auto interp = (val1 * (numPoints - i) + val2 * (i)) / numPoints;
						buffer.push_back(interp);
					}
					lastPointTime += preci * numPoints;
				}
			}

			double precision() const { return graph->timeWindow / MAX_POINTS; }

			/// Minimum/maximum vertical values of graph in visible window.
			/// May return equal values.
			Range yRange() const {
				if (fixedRange) return fixedRange.get();
				// TODO: Limit values to visible window only.
				Range result = { 0, 0 };
				if (buffer.empty()) return result;
				result.first = result.second = buffer.front();
				for (auto y : buffer) {
					result.first = std::min(result.first, y);
					result.second = std::max(result.second, y);
				}
				return result;
			}

			Points getPoints() const {
				auto preci = precision();
				Points result;
				if (buffer.empty()) return result;
				double startTime = lastPointTime - (buffer.size() - 1) * preci;
				double timeIter = startTime;
				for (auto val : buffer) {
					result.push_back({ timeIter, val });
					timeIter += preci;
				}
				return result;
			}

			void setFixedRange(Range const& r) { fixedRange = r; }
		};

		Program04* prog = nullptr;
		Point screenSize = Point(320, 240);
		double currentTime = 0;
		double timeWindow = 5;
		std::map<std::string, std::shared_ptr<Channel>> channels;

		Graph(Program04 *prog) : prog(prog) {}
		Range xRange() const { return{ currentTime - timeWindow, currentTime }; }
		void render();
		void setCurrentTime(double t) { currentTime = t; }
		void addDataPoint(ChannelName const& channelName, double value) {
			auto& channel = getChannel(channelName);
			channel.addPoint(value);

			auto dxName = "dx(" + channelName + ")";
			auto iter = channels.find(dxName);
			if (iter != channels.end()) {
				auto dxChannel = iter->second;
				if (channel.buffer.size() > 1) {
					auto valIter = channel.buffer.rbegin();
					auto val2 = *valIter;
					valIter++;
					auto val1 = *valIter;
					auto slope = (val2 - val1) / channel.precision();
					addDataPoint(dxName, slope);
				}
			}
		}
		void setFixedRange(ChannelName const& name, Range const& r) {
			getChannel(name).setFixedRange(r);
		}
		void showChannel(ChannelName const& expr) {
			getChannel(expr);
		}

	private:
		Channel& getChannel(std::string const& name) {
			auto iter = channels.find(name);
			if (iter == channels.end()) {
				auto channel = new Channel(this);
				// TODO: give channel new color
				channels[name].reset(channel);
				return *channel;
			}
			return *(iter->second);
		}
	};
	
	Graph graph = Graph(this);

	class Motor
	{
	public:
		enum class Mode { Free, Velocity, Position };

		Physics *physics = nullptr;         // reference to parent 
		btHingeConstraint* hinge = nullptr; // reference to internal implementation
		double targetVelocity = 0;          // [state] target velocity of motor
		double targetPosition = 0;
		Mode   mode = Mode::Free;
		int cbId = 0;

		Motor(Physics *physics, btHingeConstraint* hinge) 
			: physics(physics), hinge(hinge) {
			cbId = physics->registerPreTick([=](auto t) { pretick(t); });
		}

		~Motor() {
			physics->deregisterPreTick(cbId);
		}

		btScalar getAngularPosition() const {
			return hinge->getHingeAngle();
		}
		Range getAngularLimits() const {
			return{ hinge->getLowerLimit(), hinge->getUpperLimit() };
		}
		void setTargetVelocity(double vel) {
			targetVelocity = vel;
			mode = Mode::Velocity;
		}
		void setTargetPosition(double pos, double duration) {
			targetPosition = pos;
			mode = Mode::Position;
			hinge->enableMotor(true);
			hinge->setMaxMotorImpulse(1);
			hinge->setMotorTarget(targetPosition, duration);
		}

	private:
		void pretick(double timeStep) {
			switch (mode) {
			case Mode::Free:
				break;
			case Mode::Velocity:
				hinge->enableAngularMotor(true, targetVelocity, 1.0);
				break;
			case Mode::Position:
				break;
			}
		}
	};
	
	class BalanceController
	{
	public:
	} balanceController;

	Program04() :
		pickCtrl(&window.camera, &physics, &window)
	{
		window.addKeyCallback('r', [this]() { 
			printf("Restarting ...\n"); 
			physics.reset(); 
			initPhysics(); 
		});
		window.addKeyCallback('p', [this]() { 
			pauseSimulation = !pauseSimulation; 
		});
		window.addListener(&pickCtrl);
	}

	void initPhysics();
	void renderPhysics()
	{
		physics.debugDraw();
		graph.render();
	}

	void doPhysics();

	void show()
	{
		window.renderFunction = [&]() {
			if (!pauseSimulation) doPhysics();
			renderPhysics();
		};
		window.run();
	}

	static int main() {
		Program04 program;
		program.initPhysics();
		program.show();
		return 0;
	}
};


