using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using BehaviorDesigner.Runtime.Tasks;
using UnityEngine.AI;

/*
 * Accion de cerrar la puerta de la celda, yendo hacia la palanca, cuando la alcanza devuelve Success
 */

public class GhostCloseDoorAction : Action
{
    NavMeshAgent agent;
    GameBlackboard blackboard;
    GameObject puerta;
    GameObject singer;

    public override void OnAwake()
    {
        agent = GetComponent<NavMeshAgent>();
        blackboard = GameObject.FindGameObjectWithTag("Blackboard").GetComponent<GameBlackboard>();
        puerta = blackboard.puerta;
        singer = blackboard.singer;
    }

    public override TaskStatus OnUpdate()
    {
        if (blackboard.isGhostInSotano &&
               blackboard.pianoed)
            return TaskStatus.Failure;

        if (blackboard.hited)
            return TaskStatus.Failure;

        PalancaPuerta puertaScript = puerta.GetComponentInChildren<PalancaPuerta>();

        if (puertaScript.Abierta() && 
            !singer.GetComponent<Cantante>().EstaEnCelda()) 
            return TaskStatus.Success;

        if(agent.enabled)agent.SetDestination(puerta.transform.position);

        if (Vector3.SqrMagnitude(transform.position - puerta.transform.position) < 1.5f)
        {
            agent.SetDestination(transform.position);
            return TaskStatus.Success;
        }
        return TaskStatus.Running;
    }
}