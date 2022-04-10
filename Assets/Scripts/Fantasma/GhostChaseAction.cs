using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using BehaviorDesigner.Runtime.Tasks;
using UnityEngine.AI;

/*
 * Accion de seguir a la cantante, cuando la alcanza devuelve Success
 */

public class GhostChaseAction : Action
{
    NavMeshAgent agent;
    GameObject singer;
    GameObject lever;
    GameBlackboard blackboard;

    public override void OnAwake()
    {
        agent = GetComponent<NavMeshAgent>();
        singer = GameObject.FindGameObjectWithTag("Blackboard").GetComponent<GameBlackboard>().singer;
        blackboard = GameObject.FindGameObjectWithTag("Blackboard").GetComponent<GameBlackboard>();
    }

    public override TaskStatus OnUpdate()
    {
        if (blackboard.isGhostInSotano &&
            blackboard.pianoed)
            return TaskStatus.Failure;

        if (blackboard.hited)
            return TaskStatus.Failure;

        lever = blackboard.nearestLever(this.gameObject);

        ControlPalanca palancaReal = lever.GetComponentInChildren<ControlPalanca>();

        GameObject publico = palancaReal.publico;

        Publico publicoScript = publico.GetComponentInChildren<Publico>();

        if (publicoScript.getLuces()) return TaskStatus.Failure;

        if (agent.isActiveAndEnabled)agent.SetDestination(singer.transform.position);

        if (Vector3.SqrMagnitude(transform.position - singer.transform.position) < 1.5f)
        {
            agent.SetDestination(transform.position);
            singer.GetComponent<Cantante>().capturada = true;
            singer.GetComponent<Cantante>().asaltante = this.gameObject.transform;
            return TaskStatus.Success;
        }
        else return TaskStatus.Running;
    }
}
