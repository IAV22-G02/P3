using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ataque : MonoBehaviour
{
    GameBlackboard blackboard;

    public void OnTriggerEnter(Collider other)
    {
        ControlPiano piano = other.gameObject.GetComponent<ControlPiano>();
        FantasmaInfo ghost = other.gameObject.GetComponent<FantasmaInfo>();

        blackboard = GameObject.FindGameObjectWithTag("Blackboard").GetComponent<GameBlackboard>();

        if (piano != null) piano.tocado = true;
        else if (ghost != null)
        {
            blackboard.hited = true;
            Debug.Log("Espabilado");
        }
    }
}