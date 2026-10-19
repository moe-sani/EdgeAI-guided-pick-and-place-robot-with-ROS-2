# Arduino Project Hub publishing notes

The reader-facing article is [arduino-project-hub.md](arduino-project-hub.md). A [PDF reading copy](arduino-project-hub.pdf) is included; GIFs appear as representative still frames in the PDF. Use the Markdown article and original GIFs for the online version.

## Suggested project fields

| Field | Suggested value |
|---|---|
| Title | A Robot That Can Say No: Local AI on Dragonwing IQ9 |
| Short description | Use local AI to command a simulated robot arm, verify its picks, and refuse commands when a camera detects a person in the cell. |
| Project type | Showcase |
| Difficulty | Advanced |
| Primary tag | Dragonwing Robotics |
| Additional tags, where available | Robotics, ROS 2, Edge AI, Machine Learning |
| Main device | Qualcomm Dragonwing IQ-9075 EVK, quantity 1 |
| Cover | `assets/project-hub-cover.jpg` |
| Code repository | https://github.com/moe-sani/EdgeAI-guided-pick-and-place-robot-with-ROS-2 |
| Video | https://www.youtube.com/watch?v=3_KHg4WfuSY |

The hardware and software tables in the article map to Project Hub's **Devices & Components** and **Software & Tools** fields. The reproduction steps assume Ubuntu, ROS 2, and the board-specific GenieX installation are prepared as linked. Do not list the simulated ABB arm or camera as purchased hardware.

## Article and media placement

Transfer the article into **Project description**, preserving its headings, code blocks, tables, and captions. The title and short description can go in their dedicated fields. Upload the local images and insert them at the corresponding positions; repository-relative image paths will not transfer by themselves. If the editor does not import Markdown, use its equivalent text, code, image, and table blocks.

| Position | Upload | Caption/use |
|---|---|---|
| Cover / opening | [project-hub-cover.jpg](../assets/project-hub-cover.jpg) | Landscape cover assembled from the existing board photo and UI screenshot; no generated robot imagery |
| Architecture | [architecture.png](../assets/architecture.png) | Upload-ready diagram; [architecture.svg](../assets/architecture.svg) is the editable vector source |
| Language control | [llm_pick up blue cube.gif](../assets/llm_pick%20up%20blue%20cube.gif) | Pick-and-place recording |
| Check disabled | [robot hits bob.gif](../assets/robot%20hits%20bob.gif) | Simulated collision comparison |
| Check enabled | [bob falls but safety is on.gif](../assets/bob%20falls%20but%20safety%20is%20on.gif) | Camera-check demonstration; retain the caption explaining when the check runs |
| Optional interface close-up | [local_vlm_safety.png](../assets/local_vlm_safety.png) | Full-resolution screenshot showing the live camera, judged image, and verdict |
| Optional hardware photo | [dragonwing_iq9_robotics.png](../assets/dragonwing_iq9_robotics.png) | Original portrait photograph |
| Optional author image | [Moe_explaining.jpg](../assets/Moe_explaining.jpg) | Moe presenting the project |

Use Project Hub's video block with the YouTube watch URL above. The Markdown article links to the same recording. The PNG architecture diagram does not depend on Mermaid support.

## Editorial provenance

The draft was checked against `README.md`, the implementation and configuration in `src/llm_commander/`, `demo.sh`, and the recorded results in `AGENT_HANDOFF.md`. Numerical results are attributed to those earlier development tests; this documentation update did not rerun the robot or benchmark the model. The hardware photo identifies the IQ-9075 EVK.

The original blog URL returned an HTTP 403 access error during preparation, so the article links to it without quoting or claiming to adapt unseen text. The YouTube title was retrieved through its public metadata; the visual sequence was checked using the supplied assets.
