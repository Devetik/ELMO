using UnityEngine;

public class TimeGestion : MonoBehaviour
{
    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            Time.timeScale = 1f;
        }
        if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            Time.timeScale = 5f;
        }
        if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            Time.timeScale = 10f;
        } 
        if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            Time.timeScale = 20f;
        } 
        if (Input.GetKeyDown(KeyCode.Alpha0))
        {
            Time.timeScale = 0f;
        } 
        if (Input.GetKeyDown(KeyCode.Alpha9))
        {
            Time.timeScale = 0.5f;
        } 
        if (Input.GetKeyDown(KeyCode.Alpha8))
        {
            Time.timeScale = 0.1f;
        } 
        if (Input.GetKeyDown(KeyCode.Alpha7))
        {
            Time.timeScale = 0.01f;
        }
        if (Input.GetKeyDown(KeyCode.KeypadPlus))
        {
            // Augmente le timeScale pour accélérer le temps
            Time.timeScale += 0.1f;
        }

        // Vérifie si la touche '-' du pavé numérique est pressée
        if (Input.GetKeyDown(KeyCode.KeypadMinus))
        {
            // Diminue le timeScale pour ralentir le temps
            Time.timeScale -= 0.1f;
        }
    }
}
