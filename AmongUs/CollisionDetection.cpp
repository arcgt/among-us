#include "CollisionDetection.h"

void CollisionDetection::computeBroadPhase(int broadPhaseMethod) {
    // compute possible collisions
    m_overlappingBodys.clear();

	switch (broadPhaseMethod) {
  	case 0: { // none
  		for (size_t i = 0; i < m_objects.size(); i++) {
  			for (size_t j = i + 1; j < m_objects.size(); j++) {
  				m_overlappingBodys.push_back(std::make_pair(i, j));
  			}
  		}
  		break;
  	}

  	case 1: {  // AABB
          // compute bounding boxes
          std::vector<AABB> aabbs(m_objects.size()); // creating vector of AABB of size objects.size
          for (size_t i = 0; i < aabbs.size(); i++) {
              aabbs[i].computeAABB(m_objects[i]);
          }
          for (size_t i = 0; i < m_objects.size(); i++) {
              for (size_t j = i + 1; j < m_objects.size(); j++) {
                  // add pair of objects to possible collision if their bounding boxes overlap
                  if (aabbs[i].testCollision(aabbs[j])) {
                      if (i == 0 && j == 2) {
                          m_objects[i].applyForceToCOM(Eigen::Vector3d(0, 9.81, 0));
                      }
                      if (i == 1 && j == 2) {
                          m_objects[i].applyForceToCOM(Eigen::Vector3d(0, 9.81, 0));
                      }
                      m_overlappingBodys.push_back(std::make_pair(i, j));
                  }
              }
          }
          break;
      }

  	case 2: {
  		// TODO: implement other broad phases algorithm
  		break;
  	}
	}
}

void CollisionDetection::computeNarrowPhase(int narrowPhaseMethod) {
    switch (narrowPhaseMethod) {
        case 0: {
            // exhaustive
            // iterate through all pairs of possible collisions
            for (auto overlap : m_overlappingBodys) {
                std::vector<Contact> temp_contacts[2];
                // compute intersection of a with b first and intersectino
                // of b with a and store results in temp_contacts
                for (int switcher = 0; switcher < 2; switcher++) {
                    RigidObject* a = &m_objects[(!switcher) ? overlap.first : overlap.second];
                    RigidObject* b = &m_objects[(!switcher) ? overlap.second : overlap.first];

                    Eigen::MatrixXd Va, Vb;
                    Eigen::MatrixXi Fa, Fb;
                    a->getMesh(Va, Fa);
                    b->getMesh(Vb, Fb);

                    // iterate through all faces of first object
                    for (int face = 0; face < Fa.rows(); face++) {
                        // iterate through all edges of given face
                        for (size_t j = 0; j < 3; j++) {
                            int start = Fa(face, j);
                            int end = Fa(face, (j + 1) % 3);

                            // check if there is a collision
                            ContactType ct = isColliding(Va.row(start), Va.row(end), Vb, Fb);

                            // find collision and check for duplicates
                            switch (ct) {
                                case ContactType::VERTEXFACE: {
                                    auto ret = m_penetratingVertices.insert(Fa(face, j));
                                    // if not already in set
                                    if (ret.second) {
                                        Contact temp_col = findVertexFaceCollision(Va.row(Fa(face, j)), Vb, Fb);
                                        temp_col.a = a;
                                        temp_col.b = b;
                                        temp_col.type = ContactType::VERTEXFACE;
                                        temp_contacts[switcher].push_back(temp_col);
                                    }
                                    break;
                                }
                                case ContactType::EDGEEDGE: {
                                    int orderedStart = std::min(start, end);
                                    int orderedEnd = std::max(start, end);
                                    auto ret = m_penetratingEdges.insert(std::make_pair(orderedStart, orderedEnd));
                                    // if not already in set
                                    if (ret.second) {
                                        Contact temp_col = findEdgeEdgeCollision(Va.row(orderedStart), Va.row(orderedEnd), Vb, Fb);
                                        temp_col.a = a;
                                        temp_col.b = b;
                                        temp_col.type = ContactType::EDGEEDGE;
                                        temp_contacts[switcher].push_back(temp_col);
                                    }
                                    break;
                                }
                                case ContactType::NONE: {
                                    break;
                                }
                            }
                        }
                    }
                }

                // look for vertexFace
                bool found = false;
                for (int i = 0; i < 2; i++) {
                    for (auto cont : temp_contacts[i]) {
                        if (cont.type == ContactType::VERTEXFACE) {
                            m_contacts.push_back(cont);
                            found = true;
                            break;
                        }
                    }
                    if (found) {
                        break;
                    }
                }
                if (found) {
                    continue;
                }

                // take single edgeedge if possible
                if (temp_contacts[0].size() > 0 && temp_contacts[0].size() < temp_contacts[1].size()) {
                    for (int i = 0; i < temp_contacts[0].size(); i++) {
                        m_contacts.push_back(temp_contacts[0][i]);
                    }
                } else if (temp_contacts[1].size() > 0 && temp_contacts[0].size() > temp_contacts[1].size()) {
                    for (int i = 0; i < temp_contacts[1].size(); i++) {
                        m_contacts.push_back(temp_contacts[1][i]);
                    }
                } else if (temp_contacts[0].size() > 0) {
                    for (int i = 0; i < temp_contacts[0].size(); i++) {
                        m_contacts.push_back(temp_contacts[0][i]);
                    }
                } else if (temp_contacts[1].size() > 0) {
                    for (int i = 0; i < temp_contacts[1].size(); i++) {
                        m_contacts.push_back(temp_contacts[1][i]);
                    }
                }
            }
            break;
        }

      	case 1: {
      		// TODO: implement other narrow phase algorithm
          // exhaustive
          // iterate through all pairs of possible collisions
          for (auto overlap : m_overlappingBodys) {
              std::vector<Contact> temp_contacts[2];
              // compute intersection of a with b first and intersectino
              // of b with a and store results in temp_contacts
              for (int switcher = 0; switcher < 2; switcher++) {
                  RigidObject* a = &m_objects[(!switcher) ? overlap.first : overlap.second];
                  RigidObject* b = &m_objects[(!switcher) ? overlap.second : overlap.first];

                  // make AABB bounding box around object to simplify computation
                  RigidObject newA = getAABB(*a);
                  RigidObject newB = getAABB(*b);

                  Eigen::MatrixXd Va, Vb;
                  Eigen::MatrixXi Fa, Fb;

                  if (overlap.first == 0 || overlap.first == 1 || overlap.second == 0 || overlap.second == 1) {
                    //simplify avatar mesh with bounding box
                    newA.getMesh(Va, Fa);
                    newB.getMesh(Vb, Fb);
                  } else {
                    a->getMesh(Va, Fa);
                    b->getMesh(Vb, Fb);
                  }

                  // iterate through all faces of first object
                  for (int face = 0; face < Fa.rows(); face++) {
                      // iterate through all edges of given face
                      for (size_t j = 0; j < 3; j++) {
                          int start = Fa(face, j);
                          int end = Fa(face, (j + 1) % 3);

                          // check if there is a collision
                          ContactType ct = isColliding(Va.row(start), Va.row(end), Vb, Fb);

                          // find collision and check for duplicates
                          switch (ct) {
                              case ContactType::VERTEXFACE: {
                                  auto ret = m_penetratingVertices.insert(Fa(face, j));
                                  // if not already in set
                                  if (ret.second) {
                                      Contact temp_col = findVertexFaceCollision(Va.row(Fa(face, j)), Vb, Fb);
                                      temp_col.a = a;
                                      temp_col.b = b;
                                      temp_col.type = ContactType::VERTEXFACE;
                                      temp_contacts[switcher].push_back(temp_col);
                                  }
                                  break;
                              }
                              case ContactType::EDGEEDGE: {
                                  int orderedStart = std::min(start, end);
                                  int orderedEnd = std::max(start, end);
                                  auto ret = m_penetratingEdges.insert(std::make_pair(orderedStart, orderedEnd));
                                  // if not already in set
                                  if (ret.second) {
                                      Contact temp_col = findEdgeEdgeCollision(Va.row(orderedStart), Va.row(orderedEnd), Vb, Fb);
                                      temp_col.a = a;
                                      temp_col.b = b;
                                      temp_col.type = ContactType::EDGEEDGE;
                                      temp_contacts[switcher].push_back(temp_col);
                                  }
                                  break;
                              }
                              case ContactType::NONE: {
                                  break;
                              }
                          }
                      }
                  }
              }

              // look for vertexFace
              bool found = false;
              for (int i = 0; i < 2; i++) {
                  for (auto cont : temp_contacts[i]) {
                      if (cont.type == ContactType::VERTEXFACE) {
                          m_contacts.push_back(cont);
                          found = true;
                          break;
                      }
                  }
                  if (found) {
                      break;
                  }
              }
              if (found) {
                  continue;
              }

              // take single edgeedge if possible
              if (temp_contacts[0].size() > 0 && temp_contacts[0].size() < temp_contacts[1].size()) {
                  for (int i = 0; i < temp_contacts[0].size(); i++) {
                      m_contacts.push_back(temp_contacts[0][i]);
                  }
              } else if (temp_contacts[1].size() > 0 && temp_contacts[0].size() > temp_contacts[1].size()) {
                  for (int i = 0; i < temp_contacts[1].size(); i++) {
                      m_contacts.push_back(temp_contacts[1][i]);
                  }
              } else if (temp_contacts[0].size() > 0) {
                  for (int i = 0; i < temp_contacts[0].size(); i++) {
                      m_contacts.push_back(temp_contacts[0][i]);
                  }
              } else if (temp_contacts[1].size() > 0) {
                  for (int i = 0; i < temp_contacts[1].size(); i++) {
                      m_contacts.push_back(temp_contacts[1][i]);
                  }
              }
          }
          break;
      	}
    }
}

void CollisionDetection::applyImpulse(double eps) {
    // compute impulse for all contacts
    for (auto contact : m_contacts) {
        Eigen::Vector3d vrel_vec = contact.a->getVelocity(contact.p) - contact.b->getVelocity(contact.p);
        double vrel = contact.n.dot(vrel_vec);
        if (vrel > 0) {
            // bodies are moving apart
            continue;
        }

    		// compute impulse and update the following momentums
    		Eigen::Vector3d numerator = -(1 + eps) * vrel_vec;
    		double t1 = contact.a->getMassInv();
    		double t2 = contact.b->getMassInv();
    		Eigen::Vector3d ra = contact.p - contact.a->getPosition();
    		Eigen::Vector3d rb = contact.p - contact.b->getPosition();
    		double t3 = contact.n.dot((contact.a->getInertiaInvWorld() * (ra.cross(contact.n))).cross(ra));
    		double t4 = contact.n.dot((contact.b->getInertiaInvWorld() * (rb.cross(contact.n))).cross(rb));

    		Eigen::Vector3d j = numerator / (t1 + t2 + t3 + t4);
    		Eigen::Vector3d force = j.dot(contact.n) * contact.n;

    		contact.a->setLinearMomentum(contact.a->getLinearMomentum() + force);
    		contact.b->setLinearMomentum(contact.b->getLinearMomentum() - force);
    }
}
