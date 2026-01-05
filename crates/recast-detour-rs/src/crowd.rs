use std::ptr::NonNull;

use recastnavigation_sys::{
    dtAllocCrowd, dtCrowd, dtCrowdAgent, dtFreeCrowd, dtQueryFilter, DT_BUFFER_TOO_SMALL,
    DT_FAILURE, DT_INVALID_PARAM, DT_OUT_OF_NODES, DT_PARTIAL_RESULT, DT_SUCCESS,
};

use crate::{Point, RecastQuery, POLYFLAGS_WALK};

bitflags::bitflags! {
    #[derive(Debug)]
    pub struct FindPointError: u32 {
        const FAILED_TO_FIND_PATH = DT_FAILURE;
        const INVALID_PARAM = DT_INVALID_PARAM;
        const BUFFER_TOO_SMALL = DT_BUFFER_TOO_SMALL;
        const OUT_OF_NODES = DT_OUT_OF_NODES;
        const PARTIAL_RESULT = DT_PARTIAL_RESULT;
    }
}

#[derive(Debug)]
pub enum Error {
    CreateCrowdError,
    CrowdInitError,
    AddAgentError,
    FindPointError(FindPointError),
    NoPolygonFound,
    MoveRequestError,
}

type Result<T> = std::result::Result<T, Error>;

// TODO: Re-expose?
pub use recastnavigation_sys::dtCrowdAgentParams;

pub struct DetourCrowd {
    crowd: NonNull<dtCrowd>,
}

impl Drop for DetourCrowd {
    fn drop(&mut self) {
        unsafe {
            dtFreeCrowd(self.crowd.as_ptr());
        }
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub struct DetourCrowdAgentIndex(i32);

impl DetourCrowd {
    pub fn new(max_agents: i32, max_agent_radius: f32, query: &mut RecastQuery) -> Result<Self> {
        let mut crowd = match NonNull::new(unsafe { dtAllocCrowd() }) {
            Some(crowd) => crowd,
            None => {
                return Err(Error::CreateCrowdError);
            }
        };

        let crowd_init_success = unsafe {
            crowd
                .as_mut()
                .init(max_agents, max_agent_radius, query.mesh.as_mut())
        };
        if !crowd_init_success {
            return Err(Error::CrowdInitError);
        }

        Ok(Self { crowd })
    }

    pub fn add_agent(
        &mut self,
        position: Point,
        params: &dtCrowdAgentParams,
    ) -> Result<DetourCrowdAgentIndex> {
        let index = unsafe {
            self.crowd
                .as_mut()
                .addAgent(position.0.as_ptr(), params as *const _)
        };
        if index >= 0 {
            Ok(DetourCrowdAgentIndex(index))
        } else {
            Err(Error::AddAgentError)
        }
    }

    pub fn update(&mut self, delta_in_seconds: f32) {
        unsafe {
            self.crowd
                .as_mut()
                .update(delta_in_seconds, std::ptr::null_mut());
        }
    }

    pub fn get_agent<'a>(
        &'a mut self,
        index: DetourCrowdAgentIndex,
    ) -> Option<DetourCrowdAgent<'a>> {
        let dt_agent = unsafe { self.crowd.as_mut().getAgent(index.0).as_ref() }?;
        if !dt_agent.active {
            return None;
        }
        Some(DetourCrowdAgent::new(dt_agent))
    }

    pub fn get_agent_mut<'a>(
        &'a mut self,
        index: DetourCrowdAgentIndex,
    ) -> Option<DetourCrowdAgentMut<'a>> {
        let dt_agent = unsafe { self.crowd.as_mut().getEditableAgent(index.0).as_mut() }?;
        if !dt_agent.active {
            return None;
        }
        Some(DetourCrowdAgentMut::new(dt_agent))
    }

    pub fn remove_agent(&mut self, index: DetourCrowdAgentIndex) {
        unsafe {
            self.crowd.as_mut().removeAgent(index.0);
        }
    }

    pub fn request_move_target(
        &mut self,
        query: &RecastQuery,
        index: DetourCrowdAgentIndex,
        pos: Point,
        r: f32,
    ) -> Result<()> {
        let poly = {
            let filter = dtQueryFilter {
                m_areaCost: [1.0; 64],
                m_includeFlags: POLYFLAGS_WALK,
                m_excludeFlags: 0,
            };
            let mut result_poly = 0;
            let mut result_pos = [0.0; 3];
            let status = unsafe {
                query.q.as_ref().findNearestPoly(
                    pos.0.as_ptr(),
                    [r, r, r].as_ptr(),
                    &filter,
                    &mut result_poly,
                    result_pos.as_mut_ptr(),
                )
            };
            if status != DT_SUCCESS {
                return Err(Error::FindPointError(FindPointError::from_bits_retain(
                    status,
                )));
            }
            if result_poly == 0 {
                return Err(Error::NoPolygonFound);
            }
            result_poly
        };

        let success = unsafe {
            self.crowd
                .as_mut()
                .requestMoveTarget(index.0, poly, pos.0.as_ptr())
        };
        if !success {
            return Err(Error::MoveRequestError);
        }

        Ok(())
    }

    pub fn reset_move_target(&mut self, index: DetourCrowdAgentIndex) -> Result<()> {
        let success = unsafe { self.crowd.as_mut().resetMoveTarget(index.0) };
        if !success {
            return Err(Error::MoveRequestError);
        }
        Ok(())
    }

    pub fn update_agent_params(
        &mut self,
        index: DetourCrowdAgentIndex,
        params: &dtCrowdAgentParams,
    ) {
        unsafe {
            self.crowd
                .as_mut()
                .updateAgentParameters(index.0, params as *const _);
        }
    }

    pub fn get_agent_params(&mut self, index: DetourCrowdAgentIndex) -> Option<dtCrowdAgentParams> {
        let agent = self.get_agent(index)?;
        Some(agent.agent.params)
    }
}

pub struct DetourCrowdAgent<'a> {
    agent: &'a dtCrowdAgent,
}

impl<'a> DetourCrowdAgent<'a> {
    fn new(agent: &'a dtCrowdAgent) -> Self {
        Self { agent }
    }
}

pub struct DetourCrowdAgentMut<'a> {
    agent: &'a mut dtCrowdAgent,
}

impl<'a> DetourCrowdAgentMut<'a> {
    fn new(agent: &'a mut dtCrowdAgent) -> Self {
        Self { agent }
    }

    pub fn set_pos(&mut self, pos: Point) {
        self.agent.npos = pos.0;
    }

    pub fn set_vel(&mut self, vel: Point) {
        self.agent.vel = vel.0;
    }
}

#[derive(Copy, Clone, Debug)]
pub enum DetourCrowdMoveRequestState {
    None,
    Failed,
    Valid,
    Requesting,
    WaitingForQueue,
    WaitingForPath,
    Velocity,
}

impl DetourCrowdMoveRequestState {
    fn parse(value: u8) -> Option<Self> {
        let state = match value {
            // DT_CROWDAGENT_TARGET_NONE
            0 => Self::None,
            // DT_CROWDAGENT_TARGET_FAILED
            1 => Self::Failed,
            // DT_CROWDAGENT_TARGET_VALID
            2 => Self::Valid,
            // DT_CROWDAGENT_TARGET_REQUESTING
            3 => Self::Requesting,
            // DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE
            4 => Self::WaitingForQueue,
            // DT_CROWDAGENT_TARGET_WAITING_FOR_PATH
            5 => Self::WaitingForPath,
            // DT_CROWDAGENT_TARGET_VELOCITY
            6 => Self::Velocity,
            // Unknown
            _ => return None,
        };
        Some(state)
    }
}

pub trait DetourCrowdAgentRef {
    fn pos(&self) -> Point;
    // Obviously not a point... maybe I'll switch over to glam
    fn vel(&self) -> Point;
    fn target_state(&self) -> DetourCrowdMoveRequestState;
}

impl<'a> DetourCrowdAgentRef for DetourCrowdAgent<'a> {
    fn pos(&self) -> Point {
        self.agent.npos.into()
    }

    fn vel(&self) -> Point {
        self.agent.vel.into()
    }

    fn target_state(&self) -> DetourCrowdMoveRequestState {
        DetourCrowdMoveRequestState::parse(self.agent.targetState).unwrap()
    }
}

impl<'a> DetourCrowdAgentRef for DetourCrowdAgentMut<'a> {
    fn pos(&self) -> Point {
        self.agent.npos.into()
    }

    fn vel(&self) -> Point {
        self.agent.vel.into()
    }

    fn target_state(&self) -> DetourCrowdMoveRequestState {
        DetourCrowdMoveRequestState::parse(self.agent.targetState).unwrap()
    }
}
