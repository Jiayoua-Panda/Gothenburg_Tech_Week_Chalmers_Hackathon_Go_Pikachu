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

## Docs

- [SKF challenge brief](docs/SKF_Analyze%20and%20compare%20learning%20methods%20suitable%20for%20industrial%20humanoids.pdf)
- [Participant schedule & practical info](docs/Participant_Schedule_Practical_Information_Hackathon_2026.pdf)
- [Challenge to Pitch crash course](docs/GTW%20Chalmers%20Hackathon%20-%20Challenge%20to%20Pitch%20Crash%20Course.pdf)
