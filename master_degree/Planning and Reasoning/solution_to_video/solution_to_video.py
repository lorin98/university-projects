import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Arc
import datetime
import cv2
import math
import glob
import os
import argparse
from tqdm import tqdm

class HParams():
    # benchmark_name: used to draw the obstacles
    benchmark_name = ''
    # wheter to print the path
    print_path = True
    # borders of the environment
    x_lim = (-3, 3)
    y_lim = (-3, 3)
    # fraction to enlarge image frame
    frame_frac = 1.0
    # disk robot param
    radius = 0.2
    # video parameters
    fps = 12
hparams = HParams()


def draw_obstacles():
    plt.gca().add_patch(Rectangle((-0.5, 0.25), angle=0, width=1, height=2.75, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Rectangle((-0.5, -3), angle=0, width=1, height=2.75, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))

    plt.gca().add_patch(Circle([-1.4, 1.45], radius=hparams.radius+0.1, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([-2, 1.65], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([-2, 0.75], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([-1.625, 0.375], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([-1.25, 0], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([-1.625, -0.375], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([-2, -0.75], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([-2, -1.65], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([-1.4, -1.45], radius=hparams.radius+0.1, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))

    plt.gca().add_patch(Circle([1.4, 1.45], radius=hparams.radius+0.1, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([2, 1.65], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([2, 0.75], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([1.625, 0.375], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([1.25, 0], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([1.625, -0.375], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([2, -0.75], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([2, -1.65], radius=hparams.radius, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Circle([1.4, -1.45], radius=hparams.radius+0.1, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))


def draw_maze():
    hparams.x_lim = (0, 5)
    hparams.y_lim = (0, 5)

    plt.plot([1.5, 1.5], [1.5, 3.5], linewidth=0.5, color='black')
    plt.plot([1.5, 3.5], [1.5, 1.5], linewidth=0.5, color='black')
    plt.plot([1.5, 3.5], [3.5, 3.5], linewidth=0.5, color='black')
    plt.plot([3.5, 3.5], [1.5, 3], linewidth=0.5, color='black')

    plt.plot([3.5, 4], [2.5, 2.5], linewidth=0.5, color='black')

    plt.plot([1, 1], [1, 4], linewidth=0.5, color='black')
    plt.plot([1, 4], [4, 4], linewidth=0.5, color='black')
    plt.plot([4, 4], [1, 4], linewidth=0.5, color='black')
    plt.plot([1, 2.5], [1, 1], linewidth=0.5, color='black')
    plt.plot([3, 4.5], [1, 1], linewidth=0.5, color='black')

    plt.plot([0.5, 0.5], [0.5, 4.5], linewidth=0.5, color='black')
    plt.plot([0.5, 4.5], [4.5, 4.5], linewidth=0.5, color='black')
    plt.plot([4.5, 4.5], [3, 4.5], linewidth=0.5, color='black')
    plt.plot([4.5, 4.5], [0.5, 2.5], linewidth=0.5, color='black')
    plt.plot([0.5, 4.5], [0.5, 0.5], linewidth=0.5, color='black')

    plt.plot([0, 2], [5, 5], linewidth=0.5, color='black')
    plt.plot([0, 0.5], [2.5, 2.5], linewidth=0.5, color='black')
    
    plt.arrow(0.25, 1.2, 0, -0.25, head_width=0.25, color='green', fill=True)

def draw_real():
    hparams.x_lim = (0, 32)
    hparams.y_lim = (0, 24)
    
    plt.gca().set_xticks(np.arange(0, 32, 2))
    plt.gca().set_yticks(np.arange(0, 24, 2))
    plt.grid(linestyle='--')
    
    plt.gca().add_patch(Rectangle((0, 0), angle=0, width=20, height=2, facecolor=(205/255., 219/255., 170/255.)))
    plt.gca().add_patch(Rectangle((0, 2), angle=0, width=14, height=4, facecolor=(205/255., 219/255., 170/255.)))
    plt.gca().add_patch(Rectangle((0, 6), angle=0, width=2, height=16, facecolor=(205/255., 219/255., 170/255.)))
    plt.gca().add_patch(Rectangle((0, 22), angle=0, width=32, height=2, facecolor=(205/255., 219/255., 170/255.)))
    plt.gca().add_patch(Rectangle((28, 14), angle=0, width=4, height=10, facecolor=(205/255., 219/255., 170/255.)))

    plt.plot([14, 20], [2, 2], linewidth=3.5, color='black')
    plt.plot([9, 18], [6, 6], linewidth=3.5, color='black')
    plt.plot([14, 14], [2, 4], linewidth=3.5, color='black')
    plt.plot([20, 20], [2, 12], linewidth=3.5, color='black')

    plt.plot([20, 28], [6, 6], linewidth=3.5, color='black')
    plt.plot([28, 28], [6, 12], linewidth=3.5, color='black')
    plt.plot([22, 28], [12, 12], linewidth=3.5, color='black')

    plt.plot([14, 18], [12, 12], linewidth=3.5, color='black')
    plt.plot([14, 14], [6, 12], linewidth=3.5, color='black')

    plt.plot([2, 6], [6, 6], linewidth=3.5, color='black')
    plt.plot([2, 2], [6, 22], linewidth=3.5, color='black')
    plt.plot([2, 12], [14, 14], linewidth=3.5, color='black')

    plt.plot([2, 28], [22, 22], linewidth=3.5, color='black')
    plt.plot([12, 12], [16, 18], linewidth=3.5, color='black')
    plt.plot([8, 14], [18, 18], linewidth=3.5, color='black')
    plt.plot([8, 8], [18, 20], linewidth=3.5, color='black')

    plt.plot([14, 14], [18, 22], linewidth=3.5, color='black')

    plt.plot([16, 18], [18, 18], linewidth=3.5, color='black')
    plt.plot([18, 18], [18, 22], linewidth=3.5, color='black')

    plt.plot([24, 28], [14, 14], linewidth=3.5, color='black')
    plt.plot([22, 22], [14, 22], linewidth=3.5, color='black')
    plt.plot([28, 28], [14, 22], linewidth=3.5, color='black')

    plt.gca().add_patch(Arc((14, 6), 4, 4, theta1=180, theta2=-90, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([14, 14], [4, 6], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((20, 6), 4, 4, theta1=90, theta2=180, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([18, 20], [6, 6], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((14, 6), 4, 4, theta1=180, theta2=-90, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([14, 14], [4, 6], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((20, 12), 4, 4, theta1=180, theta2=0, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([18, 22], [12, 12], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((28, 14), 4, 4, theta1=180, theta2=-90, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([28, 28], [12, 14], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((6, 6), 3, 3, theta1=0, theta2=90, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.gca().add_patch(Arc((9, 6), 3, 3, theta1=90, theta2=180, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([6, 9], [6, 6], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.plot([6, 6], [6, 7.5], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.plot([9, 9], [6, 7.5], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((12, 14), 4, 4, theta1=90, theta2=180, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([12, 12], [14, 16], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((8, 22), 4, 4, theta1=180, theta2=-90, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([8, 8], [20, 22], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((14, 18), 4, 4, theta1=-90, theta2=0, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([14, 16], [18, 18], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.plot([14, 14], [16, 18], linewidth=2.5, color=(137/255., 187/255., 204/255.))
    plt.gca().add_patch(Arc((22, 14), 4, 4, theta1=0, theta2=90, color=(137/255., 187/255., 204/255.), linewidth=2.5))
    plt.plot([22, 24], [14, 14], linewidth=2.5, color=(137/255., 187/255., 204/255.))

    plt.gca().add_patch(Rectangle((6.5, 10.25), angle=0, width=3, height=1.5, facecolor=(0., 0.5, 0.)))
    plt.gca().add_patch(Rectangle((2, 14), angle=0, width=2, height=0.75, facecolor='orangered'))
    plt.gca().add_patch(Rectangle((2, 13.25), angle=0, width=2, height=0.75, facecolor='orangered'))
    plt.gca().add_patch(Rectangle((14, 12), angle=-90, width=2, height=0.75, facecolor='orangered'))
    plt.gca().add_patch(Rectangle((28, 6), angle=90, width=2, height=0.75, facecolor='orangered'))
    plt.gca().add_patch(Rectangle((18, 21.25), angle=0, width=2, height=0.75, facecolor='orangered'))
    plt.gca().add_patch(Rectangle((22.75, 20), angle=90, width=2, height=0.75, facecolor='orangered'))
    plt.gca().add_patch(Rectangle((2, 18.25), angle=0, width=2, height=3.75, facecolor='palevioletred'))
    plt.gca().add_patch(Rectangle((14, 6), angle=0, width=1.45, height=1.45, facecolor='darkorchid'))
    plt.gca().add_patch(Rectangle((13, 2), angle=0, width=1, height=2, facecolor='saddlebrown'))

    plt.gca().add_patch(Circle([16, 20], radius=1.25, facecolor='paleturquoise', alpha=0.4))
    plt.gca().add_patch(Circle([11, 20], radius=1.5, facecolor='paleturquoise', alpha=0.4))
    plt.gca().add_patch(Circle([5.5, 16.25], radius=2, facecolor='lightcoral', alpha=0.2))
    plt.gca().add_patch(Circle([25, 19], radius=1.75, facecolor='lightcoral', alpha=0.2))
    plt.gca().add_patch(Circle([24, 9.5], radius=1.75, facecolor='lightcoral', alpha=0.2))

    plt.gca().add_patch(Rectangle((2, 6), angle=0, width=1.25, height=0.75, linewidth=1.5, edgecolor='red', fill=False))

    plt.text(6.5, 8, 'LivingRoom')
    plt.text(15, 4, 'WetKitchen')
    plt.text(15, 9, 'Kitchen')
    plt.text(22.5, 9.5, 'Bedroom3')
    plt.text(22.5, 3, 'CarPorch')
    plt.text(3, 16, 'MasterBedroom')
    plt.text(10, 20, 'WC1')
    plt.text(15, 20, 'WC2')
    plt.text(23, 19, 'Bedroom2')
    plt.text(2, 1.5, 'Garden')

    plt.text(2.5, 7.5, '0', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(8, 12.5, '1', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(5, 20, '2', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(9, 18.5, '3', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(17, 18.5, '4', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(27, 20, '5', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(25, 7, '6', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(17, 7, '7', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(23, 1, '8', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(17, 3, '9', bbox=dict(facecolor='yellow', alpha=0.5))
    plt.text(1, 3, '10', bbox=dict(facecolor='yellow', alpha=0.5))

    
def solution_to_video(input_path: str,
                      video_path: str,
                      tree_path: str,
                      exp: str):
    if tree_path != '':
        tree = plt.imread(tree_path)
        print("Tree image was received.")
    
    if exp != 'obstacles' and exp != 'maze' and exp != 'real':
        print('Invalid argument for the environment. Please, retry')
        return
    hparams.benchmark_name = exp
    if exp == 'real':
    	plt.figure(figsize=(8, 6))
    
    with open(input_path,'r') as f:
        lines = f.readlines()
        
        # according to the saving modality, lines[-2] = "\n" and lines[-1] = desired goal
        # saving starting configuration and final goal to always visualize them in the plot
        # start configuration qs
        x0, y0, theta0 = list(map(float, lines[0].split()))
        # goal configuration qg
        xf, yf, thetaf = list(map(float, lines[-1].split()))
        
        # discarding "\n" and the already-stored goal
        lines = lines[:-2]
        positions = []
        
        for row in tqdm(lines):
            # clear plot
            plt.clf()

            # reading the current configuration q
            x, y, theta = list(map(float, row.split()))
            
            positions.append([x, y])
            
            # draw scenario
            if hparams.benchmark_name == 'obstacles':
                draw_obstacles()
            elif hparams.benchmark_name == 'maze':
                draw_maze()
            elif hparams.benchmark_name == 'real':
                draw_real()
                
            # adjust visible area
            plt.xlim([hparams.frame_frac*hparams.x_lim[0], hparams.frame_frac*hparams.x_lim[1]])
            plt.ylim([hparams.frame_frac*hparams.y_lim[0], hparams.frame_frac*hparams.y_lim[1]])
            plt.gca().set_aspect('equal', adjustable='box')

            # creating the path
            if hparams.print_path:
                for p in positions:
                    plt.scatter(p[0], p[1], color = 'black', s=0.5)

            # creating the current disk robot...
            plt.gca().add_patch(Circle((x,
                                        y),
                                       radius=hparams.radius,
                                       linewidth=0.5,
                                       color='r',
                                       fill=True,
                                       alpha=0.75))
            # ... plotting also qs and qg robots
            plt.gca().add_patch(Circle((x0,
                                        y0),
                                       radius=hparams.radius,
                                       linewidth=0.5,
                                       color='r',
                                       fill=False))
            plt.gca().add_patch(Circle((xf,
                                        yf),
                                       radius=hparams.radius,
                                       linewidth=0.5,
                                       color='r',
                                       fill=False))

            # this point is the center of the disk robot...
            plt.scatter(x, y, color = 'black', s=2)
            # ... plotting also qs and qg point
            plt.scatter(x0, y0, color = 'black', s=1)
            plt.scatter(xf, yf, color = 'black', s=1)
            
            plt.plot([x, x+hparams.radius*math.cos(theta)],
                    [y, y+hparams.radius*math.sin(theta)],
                    linewidth=0.5,
                    color='black')
            plt.plot([x0, x0+hparams.radius*math.cos(theta0)],
                    [y0, y0+hparams.radius*math.sin(theta0)],
                    linewidth=0.5,
                    color='black')
            plt.plot([xf, xf+hparams.radius*math.cos(thetaf)],
                    [yf, yf+hparams.radius*math.sin(thetaf)],
                    linewidth=0.5,
                    color='black')
                    
            if tree_path != '':
                plt.imshow(tree, extent=[hparams.frame_frac*hparams.x_lim[0], hparams.frame_frac*hparams.x_lim[1], hparams.frame_frac*hparams.y_lim[0], hparams.frame_frac*hparams.y_lim[1]])
            

            # saving current frame to later create the video
            savepath = str(datetime.datetime.now()).strip() + '.png'
            plt.savefig(savepath, dpi=200, bbox_inches='tight', pad_inches=0)

    print('Conversion to video...')
    
    # collecting every frame in an array
    img_array = []
    for filename in sorted(glob.glob('*.png')):
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width, height)
        img_array.append(img)
        os.remove(filename)

    # re-creating frames as a video in '.mp4' format
    out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), hparams.fps, size)
    for image in img_array:
        out.write(image)
    out.release()
    
    print('Conversion completed.')


def main():
    
    parser = argparse.ArgumentParser()
    
    # required parameters
    parser.add_argument(
        "--input_path",
        nargs='?',
        default='solution.txt',
        type=str,
        help='Path to the input txt file.'
    )
    parser.add_argument(
        "--video_path",
        nargs='?',
        default='solution_to_video.mp4',
        type=str,
        help='Path to the output video file.'
    )
    parser.add_argument(
        "--tree_path",
        nargs='?',
        default='',
        type=str,
        help='Path to the tree image.'
    )
    parser.add_argument(
        "--exp",
        nargs='?',
        default='',
        type=str,
        help='type of experiment: {obstacles, maze, real}'
    )
    
    args = parser.parse_args()
    
    print('Creating %s video from %s to %s' % (args.exp, args.input_path, args.video_path))
    
    solution_to_video(args.input_path, args.video_path, args.tree_path, args.exp)

if __name__ == '__main__':
    main()
