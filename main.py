import pygame
import time

SCREEN_HEIGHT = 800
SCREEN_WIDTH = 600

pygame.init()
screen = pygame.display.set_mode([SCREEN_HEIGHT, SCREEN_WIDTH])
pygame.display.set_caption("Engine")

st, et, dt = 0, 0, 0

running = True
while running:
    st = time.time()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    
    et = time.time()
    dt = et-st
    time.sleep(max(0, dt))
    
pygame.quit()
