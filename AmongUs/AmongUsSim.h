#include "CollisionDetection.h"
#include "Simulation.h"

#include <deque>

using namespace std;

class AmongUsSim : public Simulation {
   public:
    AmongUsSim() : Simulation(), m_collisionDetection(m_objects) { init(); }

    const int NUM_CUBES = 2;  // number of avatars

    virtual void init() override {
        // avatars
        std::string path = "amongus.off";
        for (int i = 0; i < NUM_CUBES; i++) {
            m_objects.push_back(RigidObject(path));
        }

        // walls
        path = "cube_shallow.off";
        for (int i = 0; i < 4; ++i) {
            m_objects.push_back(RigidObject(path));
        }

        // env items
        path = "door.off";
    		m_objects.push_back(RigidObject(path));
        path = "chairsleft.off";
    		m_objects.push_back(RigidObject(path));
        path = "chairsright.off";
    		m_objects.push_back(RigidObject(path));
        path = "computer.off";
    		m_objects.push_back(RigidObject(path));

        path = "cube.off";
        for (int j = 0; j < 3; ++j) {
            m_objects.push_back(RigidObject(path));
        }

        m_collisionDetection.setObjects(m_objects);

        m_dt = 1e-3 * 3;
        m_gravity << 0, -9.81, 0;
        m_mass = 1.0;
        m_showContacts = false;
        m_broadPhaseMethod = 0;
        m_narrowPhaseMethod = 0;
        m_eps = 1.0;

        reset();
    }

    // to combine two quaternions, for example rotate by q1 followed by q2, rotate by q2*q1
    // real-time movement control with esdf, uhjk, by applying force to avatar
    void goRight(int obj) {
      m_objects[obj].setForce(Eigen::Vector3d(2500,0,0));
      Eigen::Quaterniond newQuaternion = Eigen::Quaterniond(0.7071, 0, 0.7071, 0) * Eigen::Quaterniond(0.7071, -0.7071, 0, 0);
      m_objects[obj].setRotation(newQuaternion);
    }

    void goLeft(int obj) {
      m_objects[obj].setForce(Eigen::Vector3d(-2500,0,0));
      Eigen::Quaterniond newQuaternion = Eigen::Quaterniond(0.7071, 0, -0.7071, 0) * Eigen::Quaterniond(0.7071, -0.7071, 0, 0);
      m_objects[obj].setRotation(newQuaternion);
    }

    void goDown(int obj) {
      m_objects[obj].setForce(Eigen::Vector3d(0,0,2500));
      m_objects[obj].setRotation(Eigen::Quaterniond(0.7071, -0.7071, 0, 0));
    }

    void goUp(int obj) {
      m_objects[obj].setForce(Eigen::Vector3d(0,0,-2500));
      Eigen::Quaterniond newQuaternion = Eigen::Quaterniond(0, 0, 1, 0) * Eigen::Quaterniond(0.7071, -0.7071, 0, 0);
      m_objects[obj].setRotation(newQuaternion);
    }

    void goJump(int obj) {
      m_objects[obj].setForce(Eigen::Vector3d(0,2500,0));
    }

    virtual void resetMembers() override {
        for (auto &o : m_objects) {
            o.reset();
        }

        // q = w + xi + yj + zk
        //
        // a = angle to rotate
        // [x, y, z] = axis to rotate around (unit vector)
        // R = [cos(a/2), sin(a/2)*x, sin(a/2)*y, sin(a/2)*z]

        // set avatar parameters
        m_objects[0].setPosition(Eigen::Vector3d(-1.5, 0, 1));
        m_objects[0].setRotation(Eigen::Quaterniond(0.7071, -0.7071, 0, 0));

    		Eigen::RowVector3d c0(204.0 / 255.0, 0, 0);
    		Eigen::RowVector3d c1(0, 128.0 / 255.0, 204.0 / 255.0);

        for (int i = 1; i < NUM_CUBES; ++i) {
          m_objects[i].setPosition(Eigen::Vector3d(1.5*i, 0, 1));
          m_objects[i].setRotation(Eigen::Quaterniond(0.7071, -0.7071, 0, 0));
          double a = double(i+1) / NUM_CUBES;
          m_objects[i].setColors(c0*a + c1*(1 - a));
        }

        for (size_t i = 0; i < NUM_CUBES; i++) {
            m_objects[i].setMass(m_mass);
        }

        // set wall parameters
        for (int i = 0; i < 4; ++i) {
      		m_objects[i + NUM_CUBES].setScale(11);
      		m_objects[i + NUM_CUBES].setType(ObjType::STATIC);
      		m_objects[i + NUM_CUBES].setColors(Eigen::RowVector3d(0.5, 0.5, 0.5));
      		m_objects[i + NUM_CUBES].setMass(std::numeric_limits<double>::max());
    	  }

        m_objects[0 + NUM_CUBES].setPosition(Eigen::Vector3d(0, -0.11, 0));

        m_objects[1 + NUM_CUBES].setPosition(Eigen::Vector3d(-11.11, 11, 0));
        m_objects[1 + NUM_CUBES].setRotation(Eigen::Quaterniond(0.7071, 0, 0, 0.7071));

        m_objects[2 + NUM_CUBES].setPosition(Eigen::Vector3d(11.11, 11, 0));
        m_objects[2 + NUM_CUBES].setRotation(Eigen::Quaterniond(0.7071, 0, 0, 0.7071));

        m_objects[3 + NUM_CUBES].setPosition(Eigen::Vector3d(0, 11, -11.11));
        m_objects[3 + NUM_CUBES].setRotation(Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5));

        // set env item parameters
        for (int i = 4; i < 8; ++i) {
      		m_objects[i + NUM_CUBES].setScale(3);
      		m_objects[i + NUM_CUBES].setType(ObjType::STATIC);
      		m_objects[i + NUM_CUBES].setMass(std::numeric_limits<double>::max());
      	}

        m_objects[4 + NUM_CUBES].setPosition(Eigen::Vector3d(0, -0.3, 0.3)); // door
        m_objects[4 + NUM_CUBES].setColors(Eigen::RowVector3d(0.2, 0.2, 0.2));

        m_objects[5 + NUM_CUBES].setPosition(Eigen::Vector3d(0.7, -1, 0)); // left chairs
      	m_objects[5 + NUM_CUBES].setColors(Eigen::RowVector3d(0.3, 0.35, 0.4));

        m_objects[6 + NUM_CUBES].setPosition(Eigen::Vector3d(-0.7, -1, 0)); // right chairs
      	m_objects[6 + NUM_CUBES].setColors(Eigen::RowVector3d(0.3, 0.35, 0.4));

        m_objects[7 + NUM_CUBES].setPosition(Eigen::Vector3d(0, -0.9, 0)); // laptop
      	m_objects[7 + NUM_CUBES].setColors(Eigen::RowVector3d(0.7, 0.7, 0.7));

        // set env cube parameters
        for (int i = 8; i < 11; ++i) {
          m_objects[i + NUM_CUBES].setType(ObjType::STATIC);
          m_objects[i + NUM_CUBES].setColors(Eigen::RowVector3d(0.3, 0.4, 0.35));
          m_objects[i + NUM_CUBES].setMass(std::numeric_limits<double>::max());
        }

        m_objects[8 + NUM_CUBES].setScale(1);
        m_objects[8 + NUM_CUBES].setPosition(Eigen::Vector3d(-6.5, 1, -3.5));
        m_objects[8 + NUM_CUBES].setRotation(Eigen::Quaterniond(0,0,0,0));

        m_objects[9 + NUM_CUBES].setScale(1.5);
        m_objects[9 + NUM_CUBES].setPosition(Eigen::Vector3d(-7, 1.3, 5.5));
        m_objects[9 + NUM_CUBES].setRotation(Eigen::Quaterniond(0, 0.2588223, 0, 0.9659249));

        m_objects[10 + NUM_CUBES].setScale(1.5);
        m_objects[10 + NUM_CUBES].setPosition(Eigen::Vector3d(6.5, 1.3, 2.5));
        m_objects[10 + NUM_CUBES].setRotation(Eigen::Quaterniond(0, 0.2588223, 0, 0.9659249));

        // among map (whole)
        // m_objects[NUM_CUBES].setScale(3);
        // m_objects[NUM_CUBES].setType(ObjType::STATIC);
        // m_objects[NUM_CUBES].setPosition(Eigen::Vector3d(0, 0, 0));
        // m_objects[NUM_CUBES].setColors(Eigen::RowVector3d(200.0/255.0, 200.0/255.0, 200.0/255.0));
        // m_objects[NUM_CUBES].setMass(std::numeric_limits<double>::max());
        // m_objects[5 + NUM_CUBES].setRotation(Eigen::Quaterniond(0, 0, 0, 0));

        updateVars();
    }

    virtual void updateRenderGeometry() override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {
                m_renderVs.emplace_back();
                m_renderFs.emplace_back();
            }

            m_objects[i].getMesh(m_renderVs[i], m_renderFs[i]);
        }
    }

    virtual bool advance() override;

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {
                int new_id = 0;
                if (i > 0) {
                    new_id = viewer.append_mesh();
                    o.setID(new_id);
                } else {
                    o.setID(new_id);
                }

                size_t meshIndex = viewer.mesh_index(o.getID());
				if (i >= NUM_CUBES) {
					viewer.data_list[meshIndex].show_lines = true;
					viewer.data_list[meshIndex].show_faces = false;
				}
				else {
					viewer.data_list[meshIndex].show_lines = false;
				}
                viewer.data_list[meshIndex].set_face_based(true);
                viewer.data_list[meshIndex].point_size = 2.0f;
                viewer.data_list[meshIndex].clear();
            }
            size_t meshIndex = viewer.mesh_index(o.getID());

            viewer.data_list[meshIndex].set_mesh(m_renderVs[i], m_renderFs[i]);
            viewer.data_list[meshIndex].compute_normals();

            Eigen::MatrixXd color;
            o.getColors(color);
            viewer.data_list[meshIndex].set_colors(color);
        }

        if (m_showContacts) {
            // number of timesteps to keep showing collision
            int delay = 10;

            // clear old points
            viewer.data_list[1].points = Eigen::MatrixXd(0, 6);
            viewer.data_list[1].point_size = 10.0f;

            // remove expired points
            while (m_contactMemory.size() > 0 &&
                   m_contactMemory.front().second + delay < m_step) {
                m_contactMemory.pop_front();
            }

            // get new points and add them to memory
            auto contacts = m_collisionDetection.getContacts();
            for (auto &contact : contacts) {
                m_contactMemory.push_back(std::make_pair(contact, m_step));
            }

            // show points
            for (auto &contact_int_p : m_contactMemory) {
                viewer.data_list[1].add_points(
                    contact_int_p.first.p.transpose(),
                    (contact_int_p.first.type == ContactType::EDGEEDGE)
                        ? Eigen::RowVector3d(0, 1, 0)
                        : Eigen::RowVector3d(0, 0, 1));
            }
        }
    }

#pragma region SettersAndGetters
    void setMethod(int m) { m_method = m; }
    /*
     * Compute magnitude and direction of momentum and apply it to o
     */
    void updateVars() {
        Eigen::Vector3d momentum;
        momentum << std::sin(m_angle), std::cos(m_angle), 0;
        momentum *= m_force;
        m_objects[0].setLinearMomentum(momentum);
    }

    void setAngle(double a) {
        m_angle = a;
        updateVars();
    }

    void setForce(double f) {
        m_force = f;
        updateVars();
    }

    void setFriction(double f) {
        m_friction = f;
        updateVars();
    }

    void setMass(double m) { m_mass = m; }

    void showContacts(bool s) {
        if (!s) {
            m_contactMemory.clear();
        }
        m_showContacts = s;
    }

    void setBroadPhaseMethod(int m) { m_broadPhaseMethod = m; }
    void setNarrowPhaseMethod(int m) { m_narrowPhaseMethod = m; }

    void setEps(double eps) { m_eps = eps; }

    Eigen::Vector3d getKineticEnergy() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            Eigen::Vector3d rotE = 0.5 * o.getInertia().diagonal().cwiseProduct(
                                             o.getAngularVelocity());
            Eigen::Vector3d kinE =
                0.5 * o.getMass() * o.getLinearVelocity().array().square();
            res += rotE + kinE;
        }
        return res;
    }

    Eigen::Vector3d getLinearMomentum() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            res += o.getLinearMomentum();
        }
        return res;
    }

    Eigen::Vector3d getAngularMomentum() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            res += o.getAngularMomentum();
        }
        return res;
    }

#pragma endregion SettersAndGetters

   private:
    int m_method;
    double m_angle;
    double m_force;
    double m_friction;
    double m_mass;

    Eigen::Vector3d m_gravity;

    CollisionDetection m_collisionDetection;
    int m_broadPhaseMethod;
    int m_narrowPhaseMethod;
    double m_eps;

    std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
    std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering

    bool m_showContacts;
    std::deque<std::pair<Contact, int>> m_contactMemory;
};
