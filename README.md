# SKF Humanoid Hackathon

GTW Chalmers Hackathon 2026 — SKF challenge: *Analyze and compare learning methods suitable for industrial humanoids*.

## StairLab interactive concept

Open [index.html](index.html) in a browser to explore the saved StairLab interface. No build step is required. GitHub displays the HTML source; download or clone the repository to open the page locally.

- Adjust stair rise, tread depth, and usable width.
- Switch between spatial and perception views.
- Show or hide illustrative footholds.
- Use the responsive layout on desktop or mobile.

The saved starting view is perception mode, with a **20 cm rise, 40 cm tread, and 110 cm width**. Subsequent changes are remembered by the browser when local storage is available.

This is an interactive geometry and interface concept, not a physics simulation or a trained robot policy. Robot poses, sensor coverage, and footholds are illustrative; experimental metrics remain unmeasured.

Files:

- `index.html`: standalone browser export, including its display and state-storage runtime.
- `web/stairlab.fragment.html`: editable interface source, preserved from the conversation visualization with the selected starting values.

The standalone page is exported with the visualize skill's `scripts/render.py`:

```sh
python3 /path/to/visualize/scripts/render.py web/stairlab.fragment.html index.html --title "StairLab — Industrial Humanoid Learning" --force
```

## MuJoCo evaluations

- [Unitree V0 walking playback](g1_step_playback/README.md) and [test-course results](g1_course/outputs/v0/segment_results.csv): the official pretrained walking policy, evaluated without new training.
- [G1-DWAQ stair replays](artifacts/stair-replays/README.md): a separate third-party stair policy on custom corridor–stair–corridor scenes. Four videos include one successful route and three failures; the successful route used simulator ground-truth position and heading for steering, without camera perception.

- [Factory route course](artifacts/factory-course/README.md): the G1-DWAQ policy with a waypoint route follower on multi-segment layouts (corners, switchback stairs, a 12-riser descent). With ideal external localization, the 19.9 m full route reaches the goal in 22/25 start conditions; the remaining failures are all on the descending flight.
- [Camera localization error sweep](artifacts/factory-course/localization-sweep/README.md): 1025 runs of the full route with degraded camera poses. Success stays comparable to a perfect camera up to ~10 cm noise, 20 cm bias, 2 Hz and 300 ms latency; fusing on-board odometry extends that to 1 Hz and 1 s.

These use different policies and test scenes, so their results are not a controlled policy comparison.

## Docs

- [SKF challenge brief](SKF_Analyze%20and%20compare%20learning%20methods%20suitable%20for%20industrial%20humanoids.pdf)
- [Participant schedule & practical info](Participant_Schedule_Practical_Information_Hackathon_2026%20%282%29.pdf)
- [Challenge to Pitch crash course](GTW%20Chalmers%20Hackathon%20-%20Challenge%20to%20Pitch%20Crash%20Course.pdf)
- [Research papers and reported results](papers/README.md)
