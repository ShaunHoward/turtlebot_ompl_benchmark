from functools import partial
from math import cos, sin
from ompl import util as ou
from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og


def state_valid(space_info, state):
    return space_info.satisfiesBounds(state)


def propagate(start, control, duration, state):
    state.setX(start.getX() + control[0] * duration * cos(start.getYaw()))
    state.setY(start.getY() + control[0] * duration * sin(start.getYaw()))
    state.setYaw(start.getYaw() + control[1] * duration)


def plan():

    se2_space = ob.SE2StateSpace()

    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-2)
    bounds.setHigh(2)
    se2_space.setBounds(bounds)

    cspace = oc.RealVectorControlSpace(se2_space, 2)

    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-1)
    cbounds.setHigh(1)
    cspace.setBounds(cbounds)

    ss = oc.SimpleSetup(cspace)

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(partial(state_valid, ss.getSpaceInformation())))
    ss.setStatePropagator(oc.StatePropagatorFn(propagate))

    start = ob.State(se2_space)
    start().setX(-0.5)
    start().setY(0.0)
    start().setYaw(0.0)

    goal = ob.State(se2_space)
    goal().setX(1)
    goal().setY(1)
    goal().setYaw(0.0)

    ss.setStartAndGoalStates(start, goal, 0.05)

    si = ss.getSpaceInformation()

    planner = oc.RRT(si)

    ss.setPlanner(planner)

    si.setPropagationStepSize(0.1)

    solved = ss.solve(20.0)

    if solved:
        print ("Solved path problem: \n%s" % ss.getSolutionPath().printAsMatrix())

if __name__ == '__main__':
    plan()
