using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using BehaviorDesigner.Runtime.Tasks;
using UnityEngine.AI;

/*
 * Accion de encarcelar a la cantante, llevandola a la celda, cuando la encierra devuelve Success
 */

public class GhostImprisonAction : Action
{
    NavMeshAgent agent;
    GameBlackboard blackboard;
    GameObject prison;

    public override void OnAwake()
    {
        agent = GetComponent<NavMeshAgent>();
        blackboard = GameObject.FindGameObjectWithTag("Blackboard").GetComponent<GameBlackboard>();
        prison = blackboard.celda;
    }

    public override TaskStatus OnUpdate()
    {

        if (blackboard.pianoed || blackboard.hited)
            return TaskStatus.Failure;

        if (agent.enabled)
            agent.SetDestination(prison.transform.position);
        if (Vector3.SqrMagnitude(transform.position - prison.transform.position) < 1.5f)
        {
            agent.SetDestination(transform.position);
            blackboard.singer.transform.parent = null;
            blackboard.singer.GetComponent<Cantante>().capturada = false;
            blackboard.singer.GetComponent<Cantante>().asaltante = null;
            return TaskStatus.Success;
        }
        else return TaskStatus.Running;
    }
}