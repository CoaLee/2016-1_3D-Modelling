using UnityEngine;

namespace Battlehub.RTEditorDemo
{
    public class Goal : MonoBehaviour
    {

        public GameObject Target;

        private void OnTriggerEnter(Collider collider)
        {
            if (Target != null)
            {
                Target.SetActive(true);
            }
        }
    }
}

