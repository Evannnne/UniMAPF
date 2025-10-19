using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A singleton for MonoBehaviour components who cannot be initialized from within code.
/// </summary>
/// <typeparam name="T"></typeparam>
public abstract class MonoSingleton<T> : MonoBehaviour where T : MonoBehaviour
{
    private static T _instance;
    public static T Instance => _instance;

    public virtual void Awake()
    {
        if (_instance == null)
        {
            _instance = this as T;
            if (_instance == null) Debug.LogError($"Attempt to assign singleton of type {typeof(T)} attached to {gameObject.name} failed due to invalid cast! This component type is {GetType()}.");
        }
        else
        {
            Debug.LogWarning($"Attempt to assign singleton of type {typeof(T)} attached to {gameObject.name} failed as there is already an instance attached to {_instance.gameObject.name}.");
            Destroy(this);
        }
    }

    public virtual void OnDestroy()
    {
        if (_instance == this)
        {
            _instance = null;
        }
    }
}
