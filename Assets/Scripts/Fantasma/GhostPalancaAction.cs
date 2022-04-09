using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using BehaviorDesigner.Runtime.Tasks;
using UnityEngine.AI;

/*
 * Accion de accionar una palanca de los candelabros (la mas cercana), cuando la alcanza devuelve Success
 */

public class GhostPalancaAction : Action
{
    NavMeshAgent agent;
    GameObject lever;
    GameBlackboard blackboard;

    bool butacasVacias = false;

    public override void OnAwake()
    {
        agent = GetComponent<NavMeshAgent>();
        blackboard = GameObject.FindGameObjectWithTag("Blackboard").GetComponent<GameBlackboard>();
    }

    public override TaskStatus OnUpdate()
    {
        if (butacasVacias) return TaskStatus.Success;

        lever = blackboard.nearestLever(this.gameObject);
        var navHit = new NavMeshHit();
        NavMesh.SamplePosition(transform.position, out navHit, 2, NavMesh.AllAreas);
        if(agent.isActiveAndEnabled) agent.SetDestination(lever.transform.position);

        if (Vector3.SqrMagnitude(transform.position - lever.transform.position) < 1)
        {
            agent.SetDestination(transform.position);
            butacasVacias = true;
            return TaskStatus.Success;
        }
        else return TaskStatus.Running;
    }
}
