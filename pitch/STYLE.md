# Pitch style

The visual and writing style of the pitch film ([index.html](index.html)). Use it for anything the audience sees: new scenes, Canva slides, posters, result images. `index.html` is the reference implementation; when this guide and the file disagree, the file wins.

## Principles

1. **One idea per scene.** If a scene needs two headlines, it is two scenes.
2. **The number is the headline.** A measured result goes up in the largest size on screen, with its sample size underneath (`22 / 25`, `from 1025 simulated runs`).
3. **Say it in five words.** Headlines are short sentences with a full stop: *Every factory has stairs.* *Ordinary is enough.* *Train only what fails.*
4. **Show, then state.** Video or drawing first, the number after it lands.
5. **Only measured claims.** Every number on screen comes from our tests and appears in the repository. Plans and ideas are labelled as such.
6. **Black, quiet, one accent.** Most of the frame is black and white. Colour carries meaning (below) and is never decoration.
7. **Motion reveals, never decorates.** Things fade and rise into place, then hold still. The pause is the slide.

## Stage

- Fixed **1920 × 1080** stage, scaled to fit the window (letterboxed on black). Design in these pixels.
- Background pure black `#000`. No textures, no grids, no logos on content scenes.
- Left-aligned text starts at **x = 150**. Centred statements use the full width.
- Wide media frames: **1600 × 450** at x = 160, y = 320 (side-by-side pairs). Single media: **1120 × 630**. Full-bleed video: whole stage.
- Keep at least 100 px from every edge except full-bleed media.

## Colour

| Token | Hex | Meaning |
| :-- | :-- | :-- |
| `--bg` | `#000000` | stage |
| `--text` | `#f5f5f7` | headlines, primary text |
| `--grey` | `#86868b` | captions, eyebrows, second lines, "before" data |
| `--dim` | `#48484a` | inactive shapes, far limbs, backgrounds of drawings |
| `--amber` | `#ffb020` | the robot sensing or acting: height scan, position pin, route trace, "after" data |
| `--green` | `#30d158` | pass, goal reached, improvement |
| `--red` | `#ff453a` | fall, fail, regression, the loop back into training |

**Gradient** (one phrase per scene at most, usually the second line of a headline):

```css
linear-gradient(90deg, #ffd166 0%, #ff9f43 45%, #ff5e62 100%)
```

Hairlines and outlines in drawings: `rgba(245,245,247,0.16)` to `0.42`. Card and frame fill: `#0a0a0a` to `#0e0e10`.

## Type

Font stack: `"SF Pro Display", "Segoe UI Variable Display", "Segoe UI", "Inter", system-ui, sans-serif`. In Canva or slides use **Inter** (or SF Pro if available).

| Class | Size | Weight | Tracking | Use |
| :-- | :-- | :-- | :-- | :-- |
| `.xl` | 190 px (300 for a lone number) | 700 | -0.05em | the one number, the closing line |
| `.l` | 124 px | 700 | -0.045em | scene headline |
| `.m` | 84 px | 700 | -0.035em | statements, stat lines, questions |
| `.s` | 32 px, grey | 500 | -0.01em | captions, sample sizes, context |
| `.eyebrow` | 30 px, grey | 600 | +0.01em | topic label above a headline |
| video label `.vl` | 24 px | 600 | | pill on top of a video: what the viewer is looking at |
| canvas labels | 24–44 px | 500–700 | | same scale, same font |

- Line height 1.0–1.06 for headlines, 1.35 for captions.
- Tight tracking on big type is what makes the style; do not leave it at the default.
- Second line of a two-line headline is either the gradient (the point) or grey (the aside).

## Components

- **Headline pair.** White first line, gradient second line, 130 px apart at `.l` size.
- **Big number.** `.xl` in green (or white), counts up from the old value to the new one over ~1.8 s. Caption `.s` underneath says what it counts and over how many runs.
- **Before → after.** `44 → 0`, `6 → 20`. The arrow in grey, the new value in green if better, red if worse.
- **Media frame.** Rounded 30 px, 1 px white hairline at 7 %, deep shadow. Full-bleed video has no radius and no shadow.
- **Video pills.** Top-left and top-right of a pair: `camera only · 1 Hz` / `+ odometry · 1 Hz`. Black at 60 % with blur.
- **Number over video.** Dim the video with `rgba(0,0,0,.72)` or the bottom fade (`transparent → .92` black), then put the number on top.
- **Pipeline circles.** Dark discs with a hairline outline, a small line glyph inside, name at 44 px and a grey subtitle at 24 px. The final "Tests" node outlined green.
- **Failure loop.** Red dotted line (dash 2 / 12, round caps, slowly moving) from the failing stage back to training, labelled *more challenges*.
- **Charts.** No boxes, no gridlines except the axis and a dashed maximum. Grey line for "before", amber for "after", dashed amber for a second seed. Rings pulse around the points that matter: green for the gain, red for the regression. Direct labels at the line ends, no legend box.
- **Robot drawing.** White and grey limbs, amber visor and chest light, amber height-scan dots in front of the feet.

## Motion

| Effect | Used for | Behaviour |
| :-- | :-- | :-- |
| `up` (default) | text | rise 36 px and unblur 12 px while fading in; leave with 18 px lift and 8 px blur |
| `zoom` | video frames | scale 1.06 → 1.0 in, then drift +3.5 % over 12 s |
| `fade` | canvases, overlays | opacity only |

- Easing: ease-out cubic for entrances, ease-in-out cubic for lines drawing and numbers counting.
- Durations: 0.9 s in, 0.7 s out. Long fades (1.0–1.6 s) only for full-screen media and the hero.
- Stagger lines of one statement by **0.5–0.6 s**. A caption follows its number by ~0.5 s.
- Scenes **overlap by 0.3–0.5 s**: the next one starts while the last one is still fading out.
- Nothing moves while the speaker talks, except slow ambient loops (scan dots, a pulsing ring, a walking marker).

## Scene grammar

Each scene in `index.html` is a `<section class="scene">` on one timeline:

```html
<section class="scene" data-start="45.6" data-holds="1.8,6.2"
  data-notes="First pause note. || Second pause note.">
  <div class="el m full" data-in="0.2" data-out="2.4" style="top:460px">Question?</div>
  <div class="el l full" data-in="2.8" data-out="8.8" style="top:110px"><span class="grad">Answer</span> here.</div>
</section>
```

- `data-start`: scene start in seconds on the global timeline, 0.3–0.5 s before the previous scene's last element has faded out.
- `data-in` / `data-out`: element times relative to the scene start. `data-fx`, `data-din`, `data-dout` override effect and durations.
- `data-holds`: where the film pauses for the speaker (relative seconds, comma-separated). One hold per beat.
- `data-notes`: speaker notes, one per hold, separated by `||`.
- Canvas drawings: `<canvas class="el" data-draw="name" data-w=".." data-h="..">` plus a function in `DRAW`.
- Numbers that count: `data-count="36,99" data-cat="2.0" data-cdur="1.8" data-suffix="%"`.

**Adding a scene:** pick the one idea, write the headline in five words, pick the one number and its sample size, choose one visual (video, drawing, or nothing), set holds where you would pause, write the notes, then shift `data-start` of every later scene.

**Time budget: 3 minutes.** Speaker notes are the script; at a calm pace (about 2.3 words per second) the whole film holds about **360 words**. Adding a scene means cutting words elsewhere.

Controls while presenting: → / space / click = next beat, ← = back, **N** notes, **T** timer (turns red at 3:00), **F** fullscreen, **R** restart, `?t=45` jumps to 45 s for rehearsal.

## Words

- Sentence case everywhere. Headlines end with a full stop; eyebrows and captions do not.
- Plain words over jargon on screen (*It sees the step.*, not *terrain-aware policy switching*). Technical detail goes in the notes.
- Units with a space: `15 cm`, `8 kg`, `2 Hz`, `300 ms`. Ratios as `22 / 25`. Separators ` · `.
- Say what was tested and how often: *across 220 test runs*, *from 1025 simulated runs*.
- Name third-party work in the footer of the last scene (e.g. *Stair skill: third-party G1-DWAQ (BSD-3)*).

## Outside the film (Canva, slides, images)

Same black background, same colours, Inter Bold with tight tracking, one idea per slide, left margin about 8 % of the width, the gradient on one phrase at most. Export charts and diagrams on black with the palette above, and no drop shadows except behind media.
