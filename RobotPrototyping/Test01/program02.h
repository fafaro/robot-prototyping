#pragma once

int program02() {
	class URDFData {
	public:
		class Joint;

		class Geometry {
		public:
			class Box {
			public:
				glm::vec3 size;
			};
		};

		class Link
		{
		public:
			std::string name;
			Link *parent;
			std::list<Joint> joints;

			struct Collision {
				glm::mat4 origin;
				boost::any geometry;
			};
			boost::optional<Collision> collision;

			Link(std::string name) : name(name), parent(nullptr) {}
		};

		class Joint
		{
		public:
			Link *child;
			glm::mat4 origin;
			glm::vec3 axis;
		};

		class LinkPool : public std::map<std::string, Link*>
		{
		public:
			Link* get(std::string const& name)
			{
				auto x = this->find(name);
				if (x != this->end()) return x->second;
				return ((*this)[name] = new Link(name));
			}

			Link* getRoot()
			{
				for (auto& entry : *this)
					if (!entry.second->parent)
						return entry.second;
				return nullptr;
			}

			~LinkPool() {
				for (auto& entry : *this)
					delete entry.second;
				this->clear();
			}
		};
	};

	class URDFDoc : private tinyxml2::XMLDocument
	{
	public:
		class JointElement
		{
		private:
			tinyxml2::XMLElement *elem;
		public:
			JointElement(tinyxml2::XMLElement* elem) : elem(elem) {}
			std::string name() const { return elem->Attribute("name"); }
			std::string parent() const {
				return elem->FirstChildElement("parent")->Attribute("link");
			}
			std::string child() const {
				return elem->FirstChildElement("child")->Attribute("link");
			}
			glm::mat4 origin() const {
				return originFromElement(elem->FirstChildElement("origin"));
			}

			boost::optional<glm::vec3> axis() const {
				if (auto axisElem = elem->FirstChildElement("axis"))
					return strToVec3(axisElem->Attribute("xyz"));
				return{};
			}
		};

		class LinkElement
		{
		public:
			tinyxml2::XMLElement *elem;
			LinkElement(tinyxml2::XMLElement *elem) : elem(elem) {}

			std::string name() const { return elem->Attribute("name"); }
			bool hasCollision() const { return elem->FirstChildElement("collision") != nullptr; }
			auto getCollisionGeometry() const
			{
				return elem
					->FirstChildElement("collision")
					->FirstChildElement("geometry")
					->FirstChildElement();
			}
			glm::mat4 getCollisionOrigin() const
			{
				return originFromElement(elem
					->FirstChildElement("collision")
					->FirstChildElement("origin"));
			}
		};

		URDFDoc(std::string const& s)
		{
			LoadFile(s.c_str());
		}

	private:
		auto allElementsByTag(std::string const& name) {
			class Iterator {
			public:
				typedef tinyxml2::XMLElement element_type;
				tinyxml2::XMLElement *element;
				std::string name;

				Iterator(tinyxml2::XMLElement *p, std::string const& name) : element(p), name(name) {}
				void operator++() { element = element->NextSiblingElement(name.c_str()); }
				bool operator!=(Iterator const& rhs) const { return element != rhs.element; }
				auto operator*() { return element; }
			};
			class Iterable {
			public:
				typedef Iterator iterator_type;
				URDFDoc *doc;
				std::string name;

				Iterable(URDFDoc *doc, std::string const& name) : doc(doc), name(name) {}
				Iterator begin() {
					return Iterator(doc->RootElement()->FirstChildElement(name.c_str()), name);
				}
				Iterator end() {
					return Iterator(0, name);
				}
			} iterable(this, name);
			return iterable;
		}

	public:
		auto allJoints() { return allElementsByTag("joint"); }
		auto allLinks() { return allElementsByTag("link"); }

		static glm::vec3 strToVec3(std::string const& s) {
			double x, y, z;
			std::istringstream iss(s);
			iss >> x >> y >> z;
			return glm::vec3(x, y, z);
		}

		static glm::mat4 originFromElement(tinyxml2::XMLElement *elem)
		{
			glm::mat4 transform;
			if (auto rpyText = elem->Attribute("rpy")) {
				auto rpy = strToVec3(rpyText);
				//transform = glm::eulerAngleXYZ(rpy.x, rpy.y, rpy.z);
				//transform = glm::yawPitchRoll(rpy.z, rpy.y, rpy.x);
				//transform = glm::yawPitchRoll(rpy.y, rpy.x, rpy.z);
				//rpy.x += glm::half_pi<float>();
				transform = (glm::mat4)quatFromRPY(rpy);
			}
			if (auto xyzText = elem->Attribute("xyz")) {
				transform[3] = glm::vec4(strToVec3(xyzText), 1);
			}
			return transform;
		}

		static glm::quat quatFromRPY(glm::vec3 const& rpy)
		{
			glm::quat result;

			double phi = rpy.x / 2.0;
			double the = rpy.y / 2.0;
			double psi = rpy.z / 2.0;

			result.x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
			result.y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
			result.z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
			result.w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

			return glm::normalize(result);
		}
	};

	URDFDoc doc("../data/cyton_gamma_1500/cyton_gamma_1500.urdf");
	URDFData::LinkPool linkPool;

	for (auto x : doc.allJoints()) {
		auto joint = URDFDoc::JointElement(x);
		auto parent = linkPool.get(joint.parent());
		auto child = linkPool.get(joint.child());
		auto axis = joint.axis();
		parent->joints.push_back(URDFData::Joint{ child, joint.origin(), axis ? *axis : glm::vec3(0,0,0) });
		child->parent = parent;
	}
	auto rootLink = linkPool.getRoot();
	std::cout << rootLink->name;

	for (auto linkElemInner : doc.allLinks()) {
		auto linkElem = URDFDoc::LinkElement(linkElemInner);
		auto link = linkPool.get(linkElem.name());
		if (linkElem.hasCollision()) {
			auto collGeom = linkElem.getCollisionGeometry();
			if (collGeom->Name() == std::string("box")) {
				auto size = URDFDoc::strToVec3(collGeom->Attribute("size"));
				link->collision = URDFData::Link::Collision{
					linkElem.getCollisionOrigin(),
					URDFData::Geometry::Box{ size }
				};
			}
		}
	}

	render([&]() {
		std::function<void(URDFData::Link*)> drawLink = [&](URDFData::Link *link) {
			glMatrixMode(GL_MODELVIEW);

			GLUtil::drawAxes();
			if (link->collision) {
				glPushMatrix();
				glMultMatrixf(glm::value_ptr(link->collision->origin));
				//glRotatef(90, 1, 0, 0);

				if (link->collision->geometry.type() == typeid(URDFData::Geometry::Box)) {
					auto box = boost::any_cast<URDFData::Geometry::Box>(link->collision->geometry);
					GLUtil::drawBox(box.size);
				}
				glPopMatrix();
			}

			for (auto joint : link->joints) {
				GLUtil::drawLine(glm::vec3(0, 0, 0), joint.origin[3]);

				glPushMatrix();
				glMultMatrixf(glm::value_ptr(joint.origin));
				GLUtil::drawLine(glm::vec3(0, 0, 0), joint.axis, glm::vec3(1, 0, 1));
				drawLink(joint.child);
				glPopMatrix();
			}
		};
		drawLink(rootLink);
	});
	return 0;
}

