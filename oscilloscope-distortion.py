import matplotlib.pyplot as plt
import numpy as np
import scipy.io.wavfile as wf

class LineSegment:
   def __init__(self,p1,p2):
      self.p1 = p1
      self.p2 = p2

      # since we expect to use these values a lot,
      # precalculate them now
      self.vec = p2-p1
      self.normsquared = np.linalg.norm(self.vec,2)**2

      # If p1 == p2, then our normalization by dividing
      # by self.norm will break stuff
      # But since we know vec is 0 in this case, we can
      # rely on that, and lie about what the norm is here
      if(self.normsquared == 0):
         self.normsquared = 1
   
   def ClosestPoint(self,p):
      # We take the approach proposed by Joshua
      # https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
      # First, we generate a metric to tell us relatively where
      # the point p is in relation to our line sigment
      # Next, we determine which point we should find the distance
      # to on our line. This will be p1, p2, or some point
      # in between p1 and p2, depending on which is closest

      # Find the projected length of the vector from p to p1 onto
      # our line segment's vector
      proj_p_len = np.dot(self.vec,p-self.p1)

      # Normalize this length to the length of the line segment
      proj_p_norm_len = proj_p_len / self.normsquared

      # At this point, we have three cases to handle to find
      # what compare point we should use
      if(proj_p_norm_len <= 0):
         # The projected length is less than 0. This means that
         # point p is closer to the line at a point not on the
         # line segment, somewhere in the direction of p1
         cp = self.p1
      elif(proj_p_norm_len >= 1):
         # The projected length is greater than 1. This means that
         # point p is closer to the line at a point not on the
         # line segment, somewhere in the direction of p2
         cp = self.p2
      else:
         # The projected length is between 0 and 1. This means that
         # point p is closest to some point on the line segment
         # in between p1 and p2. Figure out where that location is
         cp = proj_p_norm_len * self.vec + self.p1
      
      # Finally, calculate the distance from our comparison point
      # to the point given

      return cp

class BoundingBox:
   def __init__(self,tl,br):
      # Box is defined by the top-left and bottom-right corners
      self.tl = tl
      self.br = br

      # Define the lines as well, since we'll need to snap to them
      self.top    = LineSegment(tl,np.array([br[0],tl[1]]))
      self.bottom = LineSegment(br,np.array([tl[0],br[1]]))
      self.left   = LineSegment(tl,np.array([tl[0],br[1]]))
      self.right  = LineSegment(br,np.array([br[0],tl[1]]))

   def ClosestPoint(self,p):
      if self.tl[0] <= p[0]:
         if p[0] <= self.br[0]:
            if self.br[1] <= p[1]:
               if p[1] <= self.tl[1]:
                  # point p is inside our box
                  return p
               else:
                  # point p is above our top edge
                  return self.top.ClosestPoint(p)
            else:
               # point p is below our bottom edge
               return self.bottom.ClosestPoint(p)
         else:
            # point p is right of our right edge
            return self.right.ClosestPoint(p)
      else:
         # point p is left of our left edge
         return self.left.ClosestPoint(p)

def MapValuesToClosestValid(line_segments,input_vals):
   # Assumes input_vals is a nx2 array
   output_vals = np.zeros(input_vals.shape)
   for i in range(0,input_vals.shape[0]):
      ad = np.iinfo(np.int32).max
      p = input_vals[i,:]
      for ls in line_segments:
         cp = ls.ClosestPoint(p)
         d = np.linalg.norm(cp-p,2)
         if(d < ad):
            ad = d
            output_vals[i,:] = cp
            if(0 == d):
               break
      #print('closest point point to ({}) is ({})'.format(p,cp))
   return output_vals

def PlotRenderPreview(line_segments,resolution):
   invals = np.mgrid[-1:1+resolution:resolution, -1:1+resolution:resolution].reshape(2,-1).T
   closevals = MapValuesToClosestValid(line_segments,invals)
   plt.plot(closevals[:,0],closevals[:,1],'b.')
   plt.show()
   

def main():

   #segTest = LineSegment(np.array([-1,-1]),np.array([1,1]))
   #vals = np.array([[-1,-1],[0,0],[1,1],[1,0],[0,1],[-1,1],[1,-1]])
   #distvals = MapValuesToClosestValid([segTest],vals)
   #print(distvals)
   #exit(1)

   # Set up the line segments we'll be mapping our waveform onto
   #shift = 0.6
   #scale = 0.1
   #segK1 = LineSegment(np.array([(-1.0+shift)*scale,-1.0*scale]),np.array([(-1.0+shift)*scale, 1.0*scale]))
   #segK2 = LineSegment(np.array([(-1.0+shift)*scale, 0.0*scale]),np.array([( 1.0+shift)*scale, 1.0*scale]))
   #segK3 = LineSegment(np.array([(-1.0+shift)*scale, 0.0*scale]),np.array([( 1.0+shift)*scale,-1.0*scale]))
   #line_segments = [segK1,segK2,segK3]
#
   #line_segments = []
   #for scale in np.linspace(0.1, 1, num=9):
   #   segS1 = LineSegment(np.array([-1.0*scale,-1.0*scale]),np.array([-1.0*scale, 1.0*scale]))
   #   segS2 = LineSegment(np.array([-1.0*scale, 1.0*scale]),np.array([ 1.0*scale, 1.0*scale]))
   #   segS3 = LineSegment(np.array([ 1.0*scale, 1.0*scale]),np.array([ 1.0*scale,-1.0*scale]))
   #   segS4 = LineSegment(np.array([ 1.0*scale,-1.0*scale]),np.array([-1.0*scale,-1.0*scale]))
   #   line_segments.append(segS1)
   #   line_segments.append(segS2)
   #   line_segments.append(segS3)
   #   line_segments.append(segS4)

   #unit = 0.5/15
   #b01 = BoundingBox(np.array([-6.5*unit, 6.5*unit]),np.array([ 6.5*unit, 5.5*unit]))
   #b02 = BoundingBox(np.array([-6.5*unit, 6.5*unit]),np.array([-5.5*unit,-0.5*unit]))
   #b03 = BoundingBox(np.array([-6.5*unit, 0.5*unit]),np.array([ 6.5*unit,-0.5*unit]))
   #b04 = BoundingBox(np.array([ 5.5*unit, 0.5*unit]),np.array([ 6.5*unit,-5.5*unit]))
   #b05 = BoundingBox(np.array([-6.5*unit,-5.5*unit]),np.array([ 6.5*unit,-6.5*unit]))
   #
   #b06 = BoundingBox(np.array([-4.5*unit, 4.5*unit]),np.array([ 6.5*unit, 3.5*unit]))
   #b07 = BoundingBox(np.array([-4.5*unit, 4.5*unit]),np.array([-3.5*unit, 1.5*unit]))
   #b08 = BoundingBox(np.array([-4.5*unit, 2.5*unit]),np.array([ 6.5*unit, 1.5*unit]))
   #
   #b09 = BoundingBox(np.array([-6.5*unit,-1.5*unit]),np.array([ 4.5*unit,-2.5*unit]))
   #b10 = BoundingBox(np.array([ 3.5*unit,-1.5*unit]),np.array([ 4.5*unit,-4.5*unit]))
   #b11 = BoundingBox(np.array([-6.5*unit,-3.5*unit]),np.array([ 4.5*unit,-4.5*unit]))
   #line_segments = [b01,b02,b03,b04,b05,b06,b07,b08,b09,b10,b11]
   #
   #b12 = BoundingBox(np.array([-1.0     , 1.0     ]),np.array([-7.5*unit, -1.0    ]))
   #b13 = BoundingBox(np.array([-1.0     , 1.0     ]),np.array([ 1.0     , 7.5*unit]))
   #b14 = BoundingBox(np.array([-1.0     ,-7.5*unit]),np.array([ 1.0     ,-1.0     ]))
   #b15 = BoundingBox(np.array([ 7.5*unit, 1.0     ]),np.array([ 1.0     ,-1.0     ]))
   #line_segments.append(b12)
   #line_segments.append(b13)
   #line_segments.append(b14)
   #line_segments.append(b15)

   loops = [
            [
               #J
               (12,119),
               (25,105),
               (28,112),
               (34,116),
               (41,114),
               (44,108),
               (44,31),
               (59,16),
               (59,111),
               (36,135),
               (27,133),
               (19,129)
            ],
            [
               #E
               (70,104),
               (70,16),
               (115,16),
               (115,32),
               (85,32),
               (85,51),
               (108,51),
               (108,69),
               (85,69),
               (85,88),
               (115,88),
               (115,104)
            ],
            [
               #N
               (125,144),
               (125,16),
               (144,16),
               (183,80),
               (183,16),
               (201,17),
               (201,108),
               (183,108),
               (144,45),
               (144,113)
            ],
            [
               #S
               (213,131),
               (213,109),
               (251,72),
               (213,72),
               (213,16),
               (276,16),
               (276,32),
               (230,32),
               (230,55),
               (276,55),
               (276,72)
            ],
            [
               #E
               (48,207),
               (48,134),
               (70,113),
               (115,113),
               (115,129),
               (64,129),
               (64,148),
               (87,148),
               (87,164),
               (64,164),
               (64,191),
               (108,191),
               (99,207)
            ],
            [
               #A
               (109,207),
               (155,113),
               (175,113),
               (213,207),
               (193,207),
               (164,138),
               (139,191),
               (178,191),
               (185,207)
            ],
            [
               #S
               (222,207),
               (216,191),
               (260,191),
               (260,167),
               (213,167),
               (213,151),
               (276,92),
               (276,113),
               (237,151),
               (276,151),
               (276,207)
            ],
            [
               #T
               (297,207),
               (297,129),
               (272,129),
               (288,113),
               (339,113),
               (339,129),
               (314,129),
               (314,207)
            ]
           ]
   width = 349
   height = 218
   scale = 1

   line_segments = []
   outer_bound = 2 #1 is our target min/max but (1,1) is further out than that when rotated 45deg
   line_segments.append(BoundingBox(np.array([-outer_bound,outer_bound]),np.array([outer_bound,0.4*scale])))
   line_segments.append(BoundingBox(np.array([-outer_bound,outer_bound]),np.array([-0.5*scale,-outer_bound])))
   line_segments.append(BoundingBox(np.array([-outer_bound,-0.4*scale]),np.array([outer_bound,-outer_bound])))
   line_segments.append(BoundingBox(np.array([0.5*scale,outer_bound]),np.array([outer_bound,-outer_bound])))
   for loop in loops:
      for i in range(0,len(loop)-1):
         p0 = np.array(loop[i])
         p1 = np.array(loop[i+1])
         #fix scale/direction
         p0 = np.array([p0[0]-(width/2),(height/2)-p0[1]])
         p0 = p0/width*scale
         p1 = np.array([p1[0]-(width/2),(height/2)-p1[1]])
         p1 = p1/width*scale
         #create segment
         ls = LineSegment(p0,p1)
         line_segments.append(ls)
      # one last line segment between first and last
      p0 = np.array(loop[-1])
      p1 = np.array(loop[0])
      #fix scale/direction
      p0 = np.array([p0[0]-(width/2),(height/2)-p0[1]])
      p0 = p0/width*scale
      p1 = np.array([p1[0]-(width/2),(height/2)-p1[1]])
      p1 = p1/width*scale
      #create segment
      ls = LineSegment(p0,p1)
      line_segments.append(ls)

   #PlotRenderPreview(line_segments,0.02)
   #exit(1)

   # Generate a random waveform
   fs_hz = 41000
   sl_s = 1
   rand_vals = np.random.rand(fs_hz*sl_s,2)
   raw = (rand_vals - 0.5) * 2
   
   # Read in a real file
   fs_hz,data = wf.read('Bumy Goldson - Everyday Another Song.wav')
   fs_hz,data = wf.read('Silva_Addict_VIP_Club.wav')
   fs_hz,data = wf.read('Jens East - Galaxies (ft. Diandra Faye).wav')
   raw = data.astype(np.double) / np.iinfo(np.int16).max

   # Only use the first 5 seconds
   #raw = raw[0:fs_hz*5,:]
   #raw = raw[int(fs_hz*(3*60+51.5)):fs_hz*(4*60+25)]
   #raw = raw[int(fs_hz*(25.5)):int(fs_hz*(30.5))]

   mins = [np.amin(raw[:,0]),np.amin(raw[:,1])]
   maxs = [np.amax(raw[:,0]),np.amax(raw[:,1])]

   print('input mins:({},{}), maxs:({},{})'.format(mins[0],mins[1],
                                                   maxs[0],maxs[1]))

   # Since most audio files will have highly-correlated left and right,
   # apply a slow rotation to the audio
   # This will have the effect of panning the audio back and forth
   f_rot_hz = 1
   raw_spin = raw
   for i in range(0,raw.shape[0]):
      t=2*np.pi*f_rot_hz*(i/fs_hz)
      rm = np.array([[np.cos(t),-np.sin(t)],
                     [np.sin(t), np.cos(t)]])
      raw_spin[i,:] = np.matmul(rm,(raw[i,:]))

   # Distort the waveform
   distorted = MapValuesToClosestValid(line_segments,raw_spin)
   noclip_scale = np.amax(distorted)
   if(noclip_scale > 1):
      print('scaling by {} to avoid clipping'.format(1/noclip_scale))
      distorted = distorted/noclip_scale
   print('output mins:({},{}), maxs:({},{})'.format(np.amin(distorted[:,0]),np.amin(distorted[:,1]),
                                                    np.amax(distorted[:,0]),np.amax(distorted[:,1])))

   # Save out the stuff
   audio_in = (raw * np.iinfo(np.int16).max).astype(np.int16)
   audio_out = (distorted * np.iinfo(np.int16).max).astype(np.int16)
   wf.write('ai.wav',fs_hz,audio_in)
   wf.write('ao.wav',fs_hz,audio_out)


if __name__ == '__main__':
   main()
   