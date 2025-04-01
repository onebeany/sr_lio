#include "state.h"

state::state()
{
    rotation = Eigen::Quaterniond::Identity();
    translation = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    ba = Eigen::Vector3d::Zero();
    bg = Eigen::Vector3d::Zero();

    coupled_rotation = Eigen::Quaterniond::Identity();
    coupled_translation = Eigen::Vector3d::Zero();
}

state::state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, 
        const Eigen::Vector3d &velocity_, const Eigen::Vector3d& ba_, const Eigen::Vector3d& bg_,
        const Eigen::Quaterniond &coupled_rotation_, const Eigen::Vector3d &coupled_translation_)
    : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_}, coupled_rotation{coupled_rotation_}, coupled_translation{coupled_translation_}
{

}

state::state(const state* state_temp, bool copy)
{
    rotation = state_temp->rotation;
    translation = state_temp->translation;
    velocity = state_temp->velocity;
    ba = state_temp->ba;
    bg = state_temp->bg;
    coupled_rotation = state_temp->coupled_rotation;
    coupled_translation = state_temp->coupled_translation;
}

void state::release()
{

}