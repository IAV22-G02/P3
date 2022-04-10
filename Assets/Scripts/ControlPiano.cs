using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Pone el piano en estado tocado cuando se colisiona otro objeto con él
 */

public class ControlPiano : MonoBehaviour
{
    public GameObject ghost;
    public bool tocado = false;

    float timeSinceLastInteraction = 0;

    float timeToStop = 5;

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.GetComponent<Cantante>() || other.gameObject.GetComponent<Player>()) return;
        tocado = false; // Solo lo hace el fantasma
    }

    public void Interact()
    {
        tocado = true;
        timeSinceLastInteraction = timeToStop;
        GetComponent<AudioSource>().Play();
    }

    public void Update()
    {
        if (timeSinceLastInteraction >= 0)
        {
            timeSinceLastInteraction--;
            tocado = false;
        }
    }
}
