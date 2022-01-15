# Oscilloscope Drawer
This program is a very hacky, very unpolished proof-of-concept
script that distorts stereo audio files so that they render
as images on an XY-mode oscilloscope. It is not fast, not
pretty (code), and not user-friendly, but good enough for what
I wanted to do with it.

# Background
Ever since I saw the [YouScope demo](https://www.youtube.com/watch?v=s1eNjUgaB-g),
I've been intrigued by audio-generated images. Whereas the soundtrack to the
YouScope demo is independent of the visualization audio, the work of
[Atom Delta's Oscillofun](https://www.youtube.com/watch?v=o4YyI6_y6kw), and later
[Jerobeam Fenderson](https://oscilloscopemusic.com/)'s entire oscilloscope music
album, took the connection further, by rendering the images with the audio being
heard. [Chris Allen](https://www.youtube.com/channel/UCSb9_amN9Oh2WJhDTwnG3NA)
also produces cool tracks of this nature. However, one of the key aspects of
these tracks is that the rendering and music are designed to be tightly coupled,
which leads to specific waveforms (often ramps and squares) due to drawing
constraints, which aren't always the most appealing of tones.

Recently, I came across a couple demos by Kenneth Nkosi Lee Smith: an
[ambient track that renders hearts](https://www.youtube.com/watch?v=RL8m5kaxVE4),
and a series of audio-visual [sketches](https://www.youtube.com/watch?v=GldvWj8t1ko).
These tracks differed in that it was clear that the music came first, and only
later was distorted to generate pictures. This means that ANY track could be
turned into an oscilloscope music track, provided you were willing to distort it.

# Theory of Operation
In XY mode, oscilloscopes traditionally have the middle of the screen set as
the origin, (0,0). For audio to map into XY, we (somewhat arbitrarily) chose
one channel to be X, and the other to by Y. Any audio that is in-phase
(e.g. monophonic) will show up on the diagonal from bottom left to top right,
since the values will track identically. However, if there are differences
in amplitude or phase, that will shift the point off of that diagonal.

To generate an image with sound, there first needs to be a target image.
In this project, there are two classes - LineSegment and BoundingBox -
which are the building blocks for creating that image. By carefully
creating a collection of these units, an image can be formed. Of
course, if you want to do any arbitrary image, these building blocks
may be too crude. Ovals, Bezzier curves, etc. could be implemented,
but I didn't need them for the designs I was interested in, so I
didn't bother.

Finally, to distort the audio, the script "simply" looks at each
audio sample in time, and remaps it to the closest point on the drawing.
This nearest-neighbor type mapping means that every audio point is moved
the smallest distance possible from the true location, thus minimizing
the total distortion, and making the track sound as close to the original
as possible.

# Tricks and Tips
If you don't have an oscilloscope that can render XY well, then
you might consider using the super-nifty tool
[XXY Oscilloscope](https://dood.al/oscilloscope/). Hit the
"[reset all]" button in the bottom right, then browse your
hard drive to play your track of choice.

Since most audio tracks I've come across are strongly correlated
between left and right channels, there's an option to "spin" the
input audio. In musical terms, this pans the track left and right,
but visually, it means that the audio will cover as much of the
oscilloscope screen as possible, since the diagonal orientation
will change. Setting f_rot_hz too high will start introducting
VERY noticeable audio changes, so I'd recommend sticking to
values under 3Hz.

Something to watch out for when spinning is
that the value (1,1) is actually sqrt(2) from the origin. This
means that if you spin it 45 degrees, you'll actually be off
the screen. To avoid clipping of this nature, leave the
noclip_scale scaling in place, and make sure your graphic
is set up to handle values outside of the nominal +/-1.0
range.

Audio waves should generally pass through 0 quite often.
Furthermore, quiet audio is near (0,0), whereas only really
loud sounds will approach the borders. This means that most
of the time, there will be at least some samples near
(0,0). Continous energy near (0,0) makes it and the
surrounding area (roughly +/- 0.3 in my experience)
prime location for any graphics you really want to shine.

If you have areas with no image building blocks, the
audio may have to jump a long ways in order to find a
valid placement. This will generate a really bright
picture, but will also greatly distort the audio.
One way to balance this is to create a small area
for your design of choice, but then fill the areas
far from your design with boxes, so that any audio
super far out will be left alone (undistorted), while
audio near your design will snap to it, giving the
desired impression.
