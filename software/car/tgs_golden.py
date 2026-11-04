"""Integer golden model. Units, rounding and update rules match the RTL contract."""
class Channel:
    def __init__(self):
        self.initialized=False;self.f=self.b=self.d=self.s=0
        self.count=self.peak=self.events=self.hits=0;self.active=False;self.stamp=0
    def step(self,reset,valid,sample,stamp):
        if reset:self.__init__();return self.words(0)
        if not valid:return self.words(0)
        x=sample*256
        f=self.f+((x-self.f)>>2) if self.initialized else x
        b=self.b if self.initialized else x
        if self.initialized and not self.active and self.hits==0 and abs((f-b)>>8)<100:
            b+=((f-b)>>6)
        d=(f-b)>>8;s=(f-self.f)>>8 if self.initialized else 0
        mag=abs(d)
        if not self.active:
            if mag>=200:
                if self.hits>=2:self.active=True;self.hits=0;self.events=(self.events+1)&0xffffffff
                else:self.hits+=1
            else:self.hits=0
        elif mag<=100:self.active=False;self.hits=0
        self.f,self.b,self.d,self.s=f,b,d,s
        self.stamp=stamp;self.count=(self.count+1)&0xffffffff
        self.peak=max(self.peak,mag);self.initialized=True
        return self.words(1)
    def words(self,updated):
        return [updated,self.f,self.b,self.d,self.s,self.stamp,self.count,self.peak,self.events,int(self.active)]
