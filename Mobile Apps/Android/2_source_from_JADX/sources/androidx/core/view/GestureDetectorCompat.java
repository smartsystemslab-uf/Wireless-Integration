package androidx.core.view;

import android.content.Context;
import android.os.Build;
import android.os.Handler;
import android.os.Message;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.VelocityTracker;
import android.view.ViewConfiguration;

public final class GestureDetectorCompat {
    private final GestureDetectorCompatImpl mImpl;

    interface GestureDetectorCompatImpl {
        boolean isLongpressEnabled();

        boolean onTouchEvent(MotionEvent motionEvent);

        void setIsLongpressEnabled(boolean z);

        void setOnDoubleTapListener(GestureDetector.OnDoubleTapListener onDoubleTapListener);
    }

    static class GestureDetectorCompatImplBase implements GestureDetectorCompatImpl {
        private static final int DOUBLE_TAP_TIMEOUT = ViewConfiguration.getDoubleTapTimeout();
        private static final int LONGPRESS_TIMEOUT = ViewConfiguration.getLongPressTimeout();
        private static final int LONG_PRESS = 2;
        private static final int SHOW_PRESS = 1;
        private static final int TAP = 3;
        private static final int TAP_TIMEOUT = ViewConfiguration.getTapTimeout();
        private boolean mAlwaysInBiggerTapRegion;
        private boolean mAlwaysInTapRegion;
        MotionEvent mCurrentDownEvent;
        boolean mDeferConfirmSingleTap;
        GestureDetector.OnDoubleTapListener mDoubleTapListener;
        private int mDoubleTapSlopSquare;
        private float mDownFocusX;
        private float mDownFocusY;
        private final Handler mHandler;
        private boolean mInLongPress;
        private boolean mIsDoubleTapping;
        private boolean mIsLongpressEnabled;
        private float mLastFocusX;
        private float mLastFocusY;
        final GestureDetector.OnGestureListener mListener;
        private int mMaximumFlingVelocity;
        private int mMinimumFlingVelocity;
        private MotionEvent mPreviousUpEvent;
        boolean mStillDown;
        private int mTouchSlopSquare;
        private VelocityTracker mVelocityTracker;

        private class GestureHandler extends Handler {
            GestureHandler() {
            }

            GestureHandler(Handler handler) {
                super(handler.getLooper());
            }

            public void handleMessage(Message msg) {
                int i = msg.what;
                if (i == 1) {
                    GestureDetectorCompatImplBase.this.mListener.onShowPress(GestureDetectorCompatImplBase.this.mCurrentDownEvent);
                } else if (i == 2) {
                    GestureDetectorCompatImplBase.this.dispatchLongPress();
                } else if (i != 3) {
                    throw new RuntimeException("Unknown message " + msg);
                } else if (GestureDetectorCompatImplBase.this.mDoubleTapListener == null) {
                } else {
                    if (!GestureDetectorCompatImplBase.this.mStillDown) {
                        GestureDetectorCompatImplBase.this.mDoubleTapListener.onSingleTapConfirmed(GestureDetectorCompatImplBase.this.mCurrentDownEvent);
                    } else {
                        GestureDetectorCompatImplBase.this.mDeferConfirmSingleTap = true;
                    }
                }
            }
        }

        GestureDetectorCompatImplBase(Context context, GestureDetector.OnGestureListener listener, Handler handler) {
            if (handler != null) {
                this.mHandler = new GestureHandler(handler);
            } else {
                this.mHandler = new GestureHandler();
            }
            this.mListener = listener;
            if (listener instanceof GestureDetector.OnDoubleTapListener) {
                setOnDoubleTapListener((GestureDetector.OnDoubleTapListener) listener);
            }
            init(context);
        }

        private void init(Context context) {
            if (context == null) {
                throw new IllegalArgumentException("Context must not be null");
            } else if (this.mListener != null) {
                this.mIsLongpressEnabled = true;
                ViewConfiguration configuration = ViewConfiguration.get(context);
                int touchSlop = configuration.getScaledTouchSlop();
                int doubleTapSlop = configuration.getScaledDoubleTapSlop();
                this.mMinimumFlingVelocity = configuration.getScaledMinimumFlingVelocity();
                this.mMaximumFlingVelocity = configuration.getScaledMaximumFlingVelocity();
                this.mTouchSlopSquare = touchSlop * touchSlop;
                this.mDoubleTapSlopSquare = doubleTapSlop * doubleTapSlop;
            } else {
                throw new IllegalArgumentException("OnGestureListener must not be null");
            }
        }

        public void setOnDoubleTapListener(GestureDetector.OnDoubleTapListener onDoubleTapListener) {
            this.mDoubleTapListener = onDoubleTapListener;
        }

        public void setIsLongpressEnabled(boolean isLongpressEnabled) {
            this.mIsLongpressEnabled = isLongpressEnabled;
        }

        public boolean isLongpressEnabled() {
            return this.mIsLongpressEnabled;
        }

        public boolean onTouchEvent(MotionEvent ev) {
            MotionEvent motionEvent;
            GestureDetector.OnDoubleTapListener onDoubleTapListener;
            boolean pointerUp;
            int id1;
            int upIndex;
            MotionEvent motionEvent2 = ev;
            int action = ev.getAction();
            if (this.mVelocityTracker == null) {
                this.mVelocityTracker = VelocityTracker.obtain();
            }
            this.mVelocityTracker.addMovement(motionEvent2);
            boolean pointerUp2 = (action & 255) == 6;
            int skipIndex = pointerUp2 ? ev.getActionIndex() : -1;
            float sumX = 0.0f;
            float sumY = 0.0f;
            int count = ev.getPointerCount();
            for (int i = 0; i < count; i++) {
                if (skipIndex != i) {
                    sumX += motionEvent2.getX(i);
                    sumY += motionEvent2.getY(i);
                }
            }
            int div = pointerUp2 ? count - 1 : count;
            float focusX = sumX / ((float) div);
            float focusY = sumY / ((float) div);
            boolean handled = false;
            int i2 = action & 255;
            if (i2 == 0) {
                boolean z = pointerUp2;
                if (this.mDoubleTapListener != null) {
                    boolean hadTapMessage = this.mHandler.hasMessages(3);
                    if (hadTapMessage) {
                        this.mHandler.removeMessages(3);
                    }
                    MotionEvent motionEvent3 = this.mCurrentDownEvent;
                    if (motionEvent3 == null || (motionEvent = this.mPreviousUpEvent) == null || !hadTapMessage || !isConsideredDoubleTap(motionEvent3, motionEvent, motionEvent2)) {
                        this.mHandler.sendEmptyMessageDelayed(3, (long) DOUBLE_TAP_TIMEOUT);
                    } else {
                        this.mIsDoubleTapping = true;
                        handled = this.mDoubleTapListener.onDoubleTap(this.mCurrentDownEvent) | false | this.mDoubleTapListener.onDoubleTapEvent(motionEvent2);
                    }
                }
                this.mLastFocusX = focusX;
                this.mDownFocusX = focusX;
                this.mLastFocusY = focusY;
                this.mDownFocusY = focusY;
                MotionEvent motionEvent4 = this.mCurrentDownEvent;
                if (motionEvent4 != null) {
                    motionEvent4.recycle();
                }
                this.mCurrentDownEvent = MotionEvent.obtain(ev);
                this.mAlwaysInTapRegion = true;
                this.mAlwaysInBiggerTapRegion = true;
                this.mStillDown = true;
                this.mInLongPress = false;
                this.mDeferConfirmSingleTap = false;
                if (this.mIsLongpressEnabled) {
                    this.mHandler.removeMessages(2);
                    int i3 = skipIndex;
                    this.mHandler.sendEmptyMessageAtTime(2, this.mCurrentDownEvent.getDownTime() + ((long) TAP_TIMEOUT) + ((long) LONGPRESS_TIMEOUT));
                }
                this.mHandler.sendEmptyMessageAtTime(1, this.mCurrentDownEvent.getDownTime() + ((long) TAP_TIMEOUT));
                return handled | this.mListener.onDown(motionEvent2);
            } else if (i2 != 1) {
                if (i2 == 2) {
                    boolean z2 = pointerUp2;
                    if (this.mInLongPress == 0) {
                        float scrollX = this.mLastFocusX - focusX;
                        float scrollY = this.mLastFocusY - focusY;
                        if (this.mIsDoubleTapping) {
                            int i4 = skipIndex;
                            return false | this.mDoubleTapListener.onDoubleTapEvent(motionEvent2);
                        } else if (this.mAlwaysInTapRegion) {
                            int deltaX = (int) (focusX - this.mDownFocusX);
                            int deltaY = (int) (focusY - this.mDownFocusY);
                            int distance = (deltaX * deltaX) + (deltaY * deltaY);
                            if (distance > this.mTouchSlopSquare) {
                                int i5 = deltaX;
                                boolean handled2 = this.mListener.onScroll(this.mCurrentDownEvent, motionEvent2, scrollX, scrollY);
                                this.mLastFocusX = focusX;
                                this.mLastFocusY = focusY;
                                this.mAlwaysInTapRegion = false;
                                this.mHandler.removeMessages(3);
                                this.mHandler.removeMessages(1);
                                this.mHandler.removeMessages(2);
                                handled = handled2;
                            }
                            if (distance > this.mTouchSlopSquare) {
                                this.mAlwaysInBiggerTapRegion = false;
                            }
                            int distance2 = skipIndex;
                            return handled;
                        } else if (Math.abs(scrollX) >= 1.0f || Math.abs(scrollY) >= 1.0f) {
                            boolean handled3 = this.mListener.onScroll(this.mCurrentDownEvent, motionEvent2, scrollX, scrollY);
                            this.mLastFocusX = focusX;
                            this.mLastFocusY = focusY;
                            int i6 = skipIndex;
                            return handled3;
                        }
                    }
                } else if (i2 == 3) {
                    boolean z3 = pointerUp2;
                    cancel();
                } else if (i2 != 5) {
                    if (i2 == 6) {
                        this.mLastFocusX = focusX;
                        this.mDownFocusX = focusX;
                        this.mLastFocusY = focusY;
                        this.mDownFocusY = focusY;
                        this.mVelocityTracker.computeCurrentVelocity(1000, (float) this.mMaximumFlingVelocity);
                        int upIndex2 = ev.getActionIndex();
                        int id12 = motionEvent2.getPointerId(upIndex2);
                        float x1 = this.mVelocityTracker.getXVelocity(id12);
                        float y1 = this.mVelocityTracker.getYVelocity(id12);
                        int i7 = action;
                        int i8 = 0;
                        while (true) {
                            if (i8 >= count) {
                                int i9 = upIndex2;
                                int i10 = id12;
                                break;
                            }
                            if (i8 == upIndex2) {
                                pointerUp = pointerUp2;
                                upIndex = upIndex2;
                                id1 = id12;
                            } else {
                                pointerUp = pointerUp2;
                                int id2 = motionEvent2.getPointerId(i8);
                                upIndex = upIndex2;
                                id1 = id12;
                                if ((this.mVelocityTracker.getXVelocity(id2) * x1) + (this.mVelocityTracker.getYVelocity(id2) * y1) < 0.0f) {
                                    int i11 = id2;
                                    this.mVelocityTracker.clear();
                                    break;
                                }
                            }
                            i8++;
                            upIndex2 = upIndex;
                            id12 = id1;
                            pointerUp2 = pointerUp;
                        }
                    } else {
                        int i12 = action;
                        boolean z4 = pointerUp2;
                    }
                } else {
                    boolean z5 = pointerUp2;
                    this.mLastFocusX = focusX;
                    this.mDownFocusX = focusX;
                    this.mLastFocusY = focusY;
                    this.mDownFocusY = focusY;
                    cancelTaps();
                }
                int i13 = skipIndex;
                return false;
            } else {
                boolean z6 = pointerUp2;
                this.mStillDown = false;
                MotionEvent currentUpEvent = MotionEvent.obtain(ev);
                if (this.mIsDoubleTapping) {
                    handled = false | this.mDoubleTapListener.onDoubleTapEvent(motionEvent2);
                } else if (this.mInLongPress) {
                    this.mHandler.removeMessages(3);
                    this.mInLongPress = false;
                } else if (this.mAlwaysInTapRegion) {
                    handled = this.mListener.onSingleTapUp(motionEvent2);
                    if (this.mDeferConfirmSingleTap && (onDoubleTapListener = this.mDoubleTapListener) != null) {
                        onDoubleTapListener.onSingleTapConfirmed(motionEvent2);
                    }
                } else {
                    VelocityTracker velocityTracker = this.mVelocityTracker;
                    int pointerId = motionEvent2.getPointerId(0);
                    velocityTracker.computeCurrentVelocity(1000, (float) this.mMaximumFlingVelocity);
                    float velocityY = velocityTracker.getYVelocity(pointerId);
                    float velocityX = velocityTracker.getXVelocity(pointerId);
                    VelocityTracker velocityTracker2 = velocityTracker;
                    if (Math.abs(velocityY) > ((float) this.mMinimumFlingVelocity) || Math.abs(velocityX) > ((float) this.mMinimumFlingVelocity)) {
                        handled = this.mListener.onFling(this.mCurrentDownEvent, motionEvent2, velocityX, velocityY);
                    }
                }
                MotionEvent motionEvent5 = this.mPreviousUpEvent;
                if (motionEvent5 != null) {
                    motionEvent5.recycle();
                }
                this.mPreviousUpEvent = currentUpEvent;
                VelocityTracker velocityTracker3 = this.mVelocityTracker;
                if (velocityTracker3 != null) {
                    velocityTracker3.recycle();
                    this.mVelocityTracker = null;
                }
                this.mIsDoubleTapping = false;
                this.mDeferConfirmSingleTap = false;
                this.mHandler.removeMessages(1);
                this.mHandler.removeMessages(2);
                int i14 = skipIndex;
                return handled;
            }
        }

        private void cancel() {
            this.mHandler.removeMessages(1);
            this.mHandler.removeMessages(2);
            this.mHandler.removeMessages(3);
            this.mVelocityTracker.recycle();
            this.mVelocityTracker = null;
            this.mIsDoubleTapping = false;
            this.mStillDown = false;
            this.mAlwaysInTapRegion = false;
            this.mAlwaysInBiggerTapRegion = false;
            this.mDeferConfirmSingleTap = false;
            if (this.mInLongPress) {
                this.mInLongPress = false;
            }
        }

        private void cancelTaps() {
            this.mHandler.removeMessages(1);
            this.mHandler.removeMessages(2);
            this.mHandler.removeMessages(3);
            this.mIsDoubleTapping = false;
            this.mAlwaysInTapRegion = false;
            this.mAlwaysInBiggerTapRegion = false;
            this.mDeferConfirmSingleTap = false;
            if (this.mInLongPress) {
                this.mInLongPress = false;
            }
        }

        private boolean isConsideredDoubleTap(MotionEvent firstDown, MotionEvent firstUp, MotionEvent secondDown) {
            if (!this.mAlwaysInBiggerTapRegion || secondDown.getEventTime() - firstUp.getEventTime() > ((long) DOUBLE_TAP_TIMEOUT)) {
                return false;
            }
            int deltaX = ((int) firstDown.getX()) - ((int) secondDown.getX());
            int deltaY = ((int) firstDown.getY()) - ((int) secondDown.getY());
            if ((deltaX * deltaX) + (deltaY * deltaY) < this.mDoubleTapSlopSquare) {
                return true;
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public void dispatchLongPress() {
            this.mHandler.removeMessages(3);
            this.mDeferConfirmSingleTap = false;
            this.mInLongPress = true;
            this.mListener.onLongPress(this.mCurrentDownEvent);
        }
    }

    static class GestureDetectorCompatImplJellybeanMr2 implements GestureDetectorCompatImpl {
        private final GestureDetector mDetector;

        GestureDetectorCompatImplJellybeanMr2(Context context, GestureDetector.OnGestureListener listener, Handler handler) {
            this.mDetector = new GestureDetector(context, listener, handler);
        }

        public boolean isLongpressEnabled() {
            return this.mDetector.isLongpressEnabled();
        }

        public boolean onTouchEvent(MotionEvent ev) {
            return this.mDetector.onTouchEvent(ev);
        }

        public void setIsLongpressEnabled(boolean enabled) {
            this.mDetector.setIsLongpressEnabled(enabled);
        }

        public void setOnDoubleTapListener(GestureDetector.OnDoubleTapListener listener) {
            this.mDetector.setOnDoubleTapListener(listener);
        }
    }

    public GestureDetectorCompat(Context context, GestureDetector.OnGestureListener listener) {
        this(context, listener, (Handler) null);
    }

    public GestureDetectorCompat(Context context, GestureDetector.OnGestureListener listener, Handler handler) {
        if (Build.VERSION.SDK_INT > 17) {
            this.mImpl = new GestureDetectorCompatImplJellybeanMr2(context, listener, handler);
        } else {
            this.mImpl = new GestureDetectorCompatImplBase(context, listener, handler);
        }
    }

    public boolean isLongpressEnabled() {
        return this.mImpl.isLongpressEnabled();
    }

    public boolean onTouchEvent(MotionEvent event) {
        return this.mImpl.onTouchEvent(event);
    }

    public void setIsLongpressEnabled(boolean enabled) {
        this.mImpl.setIsLongpressEnabled(enabled);
    }

    public void setOnDoubleTapListener(GestureDetector.OnDoubleTapListener listener) {
        this.mImpl.setOnDoubleTapListener(listener);
    }
}
