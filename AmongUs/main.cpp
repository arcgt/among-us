#include <igl/writeOFF.h>
#include "AmongUsSim.h"
#include "Gui.h"

class CollisionGui : public Gui {
   public:
    float m_angle = 0.0f;
    float m_force = 0.0f;
    float m_dt = 1e-3;
    float m_mass = 1.0;
    bool m_showContacts = false;
    float m_eps = 1.0;

    const vector<char const*> m_methods = { "Matrix", "Quaternion" };
    int m_selected_method = 0;

    int m_maxHistory = 200;
    std::vector<float> m_energy_history;

    int m_selectedBroadPhase = 1;
    const std::vector<char const *> m_broadphases = {"None", "AABB", "Own"};
    int m_selectedNarrowPhase = 1;
    const std::vector<char const *> m_narrowphases = {"Exhaustive", "Own"};

    AmongUsSim *p_AmongUsSim = NULL;

    CollisionGui() {
        p_AmongUsSim = new AmongUsSim();
        setSimulation(p_AmongUsSim);
        setFastForward(true); // no delay

        // show vertex velocity instead of normal
        callback_clicked_vertex = [&](int clickedVertexIndex,
                                      int clickedObjectIndex,
                                      Eigen::Vector3d &pos,
                                      Eigen::Vector3d &dir) {
            RigidObject &o = p_AmongUsSim->getObjects()[clickedObjectIndex];
            pos = o.getVertexPosition(clickedVertexIndex);
            dir = o.getVelocity(pos);
        };
        start();
    }

    virtual void updateSimulationParameters() override {
        p_AmongUsSim->setMethod(m_selected_method);
        p_AmongUsSim->setForce(m_force);
        p_AmongUsSim->setAngle(m_angle);
        p_AmongUsSim->setTimestep(m_dt);
        p_AmongUsSim->setMass(m_mass);
        p_AmongUsSim->setBroadPhaseMethod(m_selectedBroadPhase);
        p_AmongUsSim->setNarrowPhaseMethod(m_selectedNarrowPhase);
        p_AmongUsSim->setEps(m_eps);
    }

    virtual void clearSimulation() override {
        p_AmongUsSim->showContacts(false);
        p_AmongUsSim->showContacts(m_showContacts);
    }

    virtual void drawSimulationParameterMenu() override {
        ImGui::Combo("Method", &m_selected_method, m_methods.data(),
            m_methods.size());
        ImGui::SliderAngle("Angle", &m_angle, -180.0f, 180.0f);
        ImGui::InputFloat("Force", &m_force, 0, 0);
        ImGui::InputFloat("Mass", &m_mass, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
        if (ImGui::Checkbox("Show contacts", &m_showContacts)) {
            p_AmongUsSim->showContacts(m_showContacts);
        }
        if (ImGui::Combo("Broadphase", &m_selectedBroadPhase,
                         m_broadphases.data(), m_broadphases.size())) {
            p_AmongUsSim->setBroadPhaseMethod(m_selectedBroadPhase);
        }
        if (ImGui::Combo("Narrowphase", &m_selectedNarrowPhase,
                         m_narrowphases.data(), m_narrowphases.size())) {
            p_AmongUsSim->setNarrowPhaseMethod(m_selectedNarrowPhase);
        }
        ImGui::InputFloat("eps", &m_eps, 0, 0);
    }

    virtual void drawSimulationStats() override {
        Eigen::Vector3d E = p_AmongUsSim->getKineticEnergy();
        m_energy_history.push_back(E.cast<float>().cwiseAbs().sum());
        if (m_energy_history.size() > m_maxHistory)
            m_energy_history.erase(m_energy_history.begin(),
                                   m_energy_history.begin() + 1);
        ImGui::Text("E: %.3f, %.3f, %.3f", E(0), E(1), E(2));
        ImGui::PlotLines("Total Energy", &m_energy_history[0],
                         m_energy_history.size(), 0, NULL, 0, 1000,
                         ImVec2(0, 200));
        Eigen::Vector3d p = p_AmongUsSim->getLinearMomentum();
        ImGui::Text("M: %.3f, %.3f, %.3f", p(0), p(1), p(2));
        Eigen::Vector3d l = p_AmongUsSim->getAngularMomentum();
        ImGui::Text("L: %.3f, %.3f, %.3f", l(0), l(1), l(2));
    }

    virtual bool childKeyCallback(igl::opengl::glfw::Viewer &viewer, unsigned int key, int modifiers) override {
        switch (key) {
            case 'f':
            case 'F':
                p_AmongUsSim->goRight(0);
                return true;
            case 's':
            case 'S':
                p_AmongUsSim->goLeft(0);
                return true;
            case 'd':
            case 'D':
                p_AmongUsSim->goDown(0);
                return true;
            case 'e':
            case 'E':
                p_AmongUsSim->goUp(0);
                return true;
            case 'v':
            case 'V':
                p_AmongUsSim->goJump(0);
                return true;
            case 'k':
            case 'K':
                p_AmongUsSim->goRight(1);
                return true;
            case 'h':
            case 'H':
                p_AmongUsSim->goLeft(1);
                return true;
            case 'j':
            case 'J':
                p_AmongUsSim->goDown(1);
                return true;
            case 'u':
            case 'U':
                p_AmongUsSim->goUp(1);
                return true;
            case 'b':
            case 'B':
                p_AmongUsSim->goJump(1);
                return true;
        }
        return false;
    }
};

int main(int argc, char *argv[]) {
    new CollisionGui();

    return 0;
}
