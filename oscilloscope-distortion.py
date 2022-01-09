import numpy as np
import scipy.io.wavfile as wf

class LineSegment:
   def __init__(self,p1,p2):
      self.p1 = p1
      self.p2 = p2

      # since we expect to use these values a lot,
      # precalculate them now
      self.vec = p2-p1
      self.norm = np.linalg.norm(self.vec,2)

      # If p1 == p2, then our normalization by dividing
      # by self.norm will break stuff
      # But since we know vec is 0 in this case, we can
      # rely on that, and lie about what the norm is here
      if(self.norm == 0):
         self.norm = 1
   
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
      proj_p_norm_len = proj_p_len / self.norm

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
      #print('closest point point to ({}) is ({})'.format(p,cp))
   return output_vals

def main():
   # Set up the line segments we'll be mapping our waveform onto
   segK1 = LineSegment(np.array([-1,-1]),np.array([-1,1]))
   segK2 = LineSegment(np.array([-1,0.1]),np.array([1,1]))
   segK3 = LineSegment(np.array([-0.9,0.2]),np.array([1,-1]))
   line_segments = [segK1,segK2,segK3]
   #line_segments = [segK1]

   # Generate a random waveform
   fs_hz = 41000
   sl_s = 1
   rand_vals = np.random.rand(fs_hz*sl_s,2)
   rand_scaled = (rand_vals - 0.5) * 2

   # Distort the waveform
   rand_out = MapValuesToClosestValid(line_segments,rand_scaled)

   # Save out the stuff
   audio_in = (rand_scaled * np.iinfo(np.int16).max).astype(np.int16)
   audio_out = (rand_out * np.iinfo(np.int16).max).astype(np.int16)
   wf.write('ai.wav',fs_hz,audio_in)
   wf.write('ao.wav',fs_hz,audio_out)


if __name__ == '__main__':
   main()
   