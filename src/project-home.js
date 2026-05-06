function parseLoopNumber(value, fallback) {
    const parsed = Number(value);
    return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function safelyPlay(video) {
    const playPromise = video.play();
    if (playPromise && typeof playPromise.catch === 'function') {
        playPromise.catch(() => {});
    }
}

export function attachSmoothLoop(video) {
    if (video.dataset.smoothLoopAttached === 'true') {
        return;
    }

    video.dataset.smoothLoopAttached = 'true';

    const loopStart = parseLoopNumber(video.dataset.loopStart, 0.06);
    const loopEndOffset = parseLoopNumber(video.dataset.loopEndOffset, 0.22);
    let frameHandle = 0;
    let rafHandle = 0;

    const resetToLoopStart = () => {
        video.currentTime = loopStart;
        if (!video.paused) {
            safelyPlay(video);
        }
    };

    const maybeLoop = () => {
        const { duration, currentTime } = video;
        if (!Number.isFinite(duration) || duration <= loopStart + loopEndOffset) {
            return;
        }

        if (currentTime >= duration - loopEndOffset) {
            resetToLoopStart();
        }
    };

    const scheduleNextCheck = () => {
        if (video.paused || video.ended || frameHandle || rafHandle) {
            return;
        }

        if (typeof video.requestVideoFrameCallback === 'function') {
            frameHandle = video.requestVideoFrameCallback(() => {
                frameHandle = 0;
                maybeLoop();
                scheduleNextCheck();
            });
            return;
        }

        rafHandle = window.requestAnimationFrame(() => {
            rafHandle = 0;
            maybeLoop();
            scheduleNextCheck();
        });
    };

    video.addEventListener('loadedmetadata', () => {
        if (video.currentTime < loopStart) {
            video.currentTime = loopStart;
        }
        scheduleNextCheck();
    });
    video.addEventListener('play', scheduleNextCheck);
    video.addEventListener('timeupdate', maybeLoop);
    video.addEventListener('ended', () => {
        video.currentTime = loopStart;
        safelyPlay(video);
    });

    if (video.readyState >= HTMLMediaElement.HAVE_METADATA && video.currentTime < loopStart) {
        video.currentTime = loopStart;
    }
    if (!video.paused) {
        scheduleNextCheck();
    }
}

export function configureSmoothVideoLoops(root = document) {
    root.querySelectorAll('video[data-smooth-loop]').forEach(attachSmoothLoop);
}

configureSmoothVideoLoops();
