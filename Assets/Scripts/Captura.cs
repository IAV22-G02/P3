using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Captura : MonoBehaviour
{
    public void OnTriggerEnter(Collider other)
    {
        Cantante cant = other.gameObject.GetComponent<Cantante>();

        if(cant!= null && cant.capturada)
        {
            cant.asaltante = null;
            cant.capturada = false;
        }
        else if (cant != null && other.gameObject.CompareTag("Cantante"))
        {
            cant.asaltante = transform.parent;
            cant.capturada = true;

            Debug.Log("Has capturado a la cantante");
        }
    }
}
