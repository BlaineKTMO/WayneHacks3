using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Video;

public class VidStream : MonoBehaviour
{
    [SerializeField] private string videourl = "rtsp://localhost:8554/stream";
    private VideoPlayer videoPlayer;
    // Start is called before the first frame update
    void Awake()
    {
        videoPlayer = GetComponent<VideoPlayer>();
        if (videoPlayer)
        {
            videoPlayer.url = videourl;
            videoPlayer.playOnAwake = false;
            videoPlayer.Prepare();

            
            videoPlayer.prepareCompleted += onVideoPrepared;
        }
    }

    // Update is called once per frame
    private void onVideoPrepared(VideoPlayer source)
    {
        videoPlayer.Play();
    }
}
